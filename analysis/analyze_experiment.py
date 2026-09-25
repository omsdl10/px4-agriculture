#!/usr/bin/env python3
"""Compute documented metrics from an experiment and create summary plots."""
import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', str(Path('/tmp') / 'agriculture-matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from common import config, elevation


def rows(path):
    if not path.is_file():
        return []
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def number(row, key):
    try:
        value = row.get(key, '')
        return float(value) if value != '' else math.nan
    except (TypeError, ValueError):
        return math.nan


def distances(x, y, z):
    if len(x) < 2:
        return np.array([])
    return np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2 + np.diff(z) ** 2)


def coverage(samples, bounds, spacing):
    xmin, xmax, ymin, ymax = bounds
    resolution = max(2.0, spacing / 2)
    nx, ny = math.ceil((xmax - xmin) / resolution), math.ceil((ymax - ymin) / resolution)
    visits = np.zeros((ny, nx), dtype=np.uint16)
    radius = spacing / 2
    for x, y in samples:
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            continue
        cx, cy = int((x - xmin) / resolution), int((y - ymin) / resolution)
        reach = math.ceil(radius / resolution)
        for iy in range(max(0, cy - reach), min(ny, cy + reach + 1)):
            for ix in range(max(0, cx - reach), min(nx, cx + reach + 1)):
                gx, gy = xmin + (ix + .5) * resolution, ymin + (iy + .5) * resolution
                if math.hypot(gx - x, gy - y) <= radius:
                    visits[iy, ix] += 1
    total = max(1, visits.size)
    return {
        'method': f'{resolution:g} m raster, {radius:g} m observation radius',
        'field_coverage_percentage': 100 * np.count_nonzero(visits) / total,
        'missed_area_percentage': 100 * np.count_nonzero(visits == 0) / total,
        'overlapping_coverage_percentage': 100 * np.count_nonzero(visits > 1) / total,
    }


def analyze(experiment):
    experiment = experiment.resolve()
    analysis = experiment / 'analysis'
    analysis.mkdir(exist_ok=True)
    drones = config('drones')['drones']
    fields = {f['id']: f for f in config('fields')['fields']}
    sim = config('simulation')
    metrics = {'experiment': str(experiment), 'uavs': {}, 'mission': {}, 'multi_uav': {}, 'agriculture': {}}
    tracks = {}
    for drone in drones:
        data = rows(experiment / drone['name'].lower() / 'telemetry.csv')
        if not data:
            continue
        t = np.array([number(r, 'sim_time_s') for r in data]); host = np.array([number(r, 'host_monotonic_s') for r in data])
        x = np.array([number(r, 'x') for r in data]); y = np.array([number(r, 'y') for r in data]); z = np.array([number(r, 'z') for r in data])
        vx = np.array([number(r, 'vx') for r in data]); vy = np.array([number(r, 'vy') for r in data]); vz = np.array([number(r, 'vz') for r in data])
        speed = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
        armed = np.array([r.get('armed', '').lower() == 'true' for r in data])
        time_axis = t if np.count_nonzero(np.isfinite(t)) > 1 else host
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        flight = armed & valid
        duration = float(time_axis[flight][-1] - time_axis[flight][0]) if np.count_nonzero(flight) > 1 else 0
        volts = np.array([number(r, 'battery_voltage') for r in data]); amps = np.array([number(r, 'battery_current') for r in data])
        energy = None
        power_valid = flight & np.isfinite(volts) & np.isfinite(amps)
        if np.count_nonzero(power_valid) > 1:
            energy = float(np.trapezoid(volts[power_valid] * amps[power_valid], time_axis[power_valid]) / 3600)
        agl_error = np.array([abs(zz - elevation(xx, yy) - sim['mission_altitude_agl_m']) for xx, yy, zz in zip(x[flight], y[flight], z[flight])])
        path_distance = float(np.sum(distances(x[valid], y[valid], z[valid])))
        field = fields[drone['field']]; scale = sim['map_size_m'] / 1000
        bounds = [v * scale for v in field['bounds']]
        cover = coverage(list(zip(x[flight], y[flight])), bounds, sim.get('survey_spacing_m', 12))
        nearest_wp = None
        mission_file = experiment / drone['name'].lower() / 'mission.json'
        if mission_file.is_file():
            waypoints = json.loads(mission_file.read_text())
            wp_xy = []
            for item in waypoints:
                if item.get('frame') == 6:
                    lat, lon = item['x'] / 1e7, item['y'] / 1e7
                    wy = math.radians(lat - 47.397742) * 6378137
                    wx = math.radians(lon - 8.545594) * 6378137 * math.cos(math.radians(47.397742))
                    wp_xy.append((wx, wy))
            if wp_xy and np.any(flight):
                nearest_wp = float(np.mean([min(math.hypot(xx-wx, yy-wy) for wx,wy in wp_xy) for xx,yy in zip(x[flight],y[flight])]))
        metrics['uavs'][drone['name']] = {
            'flight_duration_s': duration,
            'total_distance_m': path_distance,
            'average_velocity_m_s': float(np.nanmean(speed[flight])) if np.any(flight) else 0,
            'maximum_velocity_m_s': float(np.nanmax(speed[flight])) if np.any(flight) else 0,
            'energy_consumption_Wh': energy,
            'energy_note': None if energy is not None else 'PX4 simulator did not publish battery current; no value was fabricated',
            'battery_consumption_percentage_points': float(np.nanmax(np.array([number(r,'battery_remaining') for r in data])[flight])-np.nanmin(np.array([number(r,'battery_remaining') for r in data])[flight])) if np.any(flight) else None,
            'mean_altitude_tracking_error_m': float(np.mean(agl_error)) if len(agl_error) else None,
            'mean_nearest_waypoint_distance_m': nearest_wp,
        }
        metrics['mission'][drone['name']] = cover
        tracks[drone['name']] = dict(t=time_axis, x=x, y=y, z=z, speed=speed, battery=np.array([number(r,'battery_remaining') for r in data]), flight=flight)

    separation = rows(experiment / 'separation.csv')
    sep_values = np.array([number(r, 'distance_m') for r in separation])
    warnings = [r for r in separation if r.get('warning', '').lower() == 'true']
    recording = json.loads((experiment / 'sensor_recording.json').read_text()) if (experiment / 'sensor_recording.json').is_file() else {}
    metrics['multi_uav'] = {
        'minimum_separation_m': float(np.nanmin(sep_values)) if len(sep_values) else None,
        'average_separation_m': float(np.nanmean(sep_values)) if len(sep_values) else None,
        'collision_warning_samples': len(warnings),
        'conflict_events': recording.get('conflict_events'),
    }
    truth = rows(experiment / 'environment/crop_ground_truth.csv')
    ndvi = np.array([number(r, 'NDVI') for r in truth])
    moisture = np.array([number(r, 'soil_moisture') for r in truth])
    healthy = sum(r.get('health_state') == 'healthy' for r in truth)
    stressed = sum('stress' in r.get('health_state', '') or 'disease' in r.get('health_state', '') for r in truth)
    inspected_area=sum((metrics['mission'][d['name']]['field_coverage_percentage']/100)*((fields[d['field']]['bounds'][1]-fields[d['field']]['bounds'][0])*sim['map_size_m']/1000)*((fields[d['field']]['bounds'][3]-fields[d['field']]['bounds'][2])*sim['map_size_m']/1000) for d in drones if d['name'] in metrics['mission'])
    metrics['agriculture'] = {
        'region_count': len(truth),
        'healthy_crop_percentage': 100 * healthy / len(truth) if truth else None,
        'stressed_crop_percentage': 100 * stressed / len(truth) if truth else None,
        'average_synthetic_NDVI': float(np.nanmean(ndvi)) if len(ndvi) else None,
        'soil_moisture_mean_percent': float(np.nanmean(moisture)) if len(moisture) else None,
        'soil_moisture_std_percent': float(np.nanstd(moisture)) if len(moisture) else None,
        'soil_moisture_range_percent': [float(np.nanmin(moisture)),float(np.nanmax(moisture))] if len(moisture) else None,
        'inspected_area_m2': inspected_area,
        'synthetic_ground_truth': True,
    }
    durations = [v['flight_duration_s'] for v in metrics['uavs'].values()]
    distances_total = [v['total_distance_m'] for v in metrics['uavs'].values()]
    metrics['mission']['fleet'] = {'total_mission_time_s': max(durations, default=0), 'total_distance_travelled_m': sum(distances_total)}
    (analysis / 'metrics.json').write_text(json.dumps(metrics, indent=2, allow_nan=False) + '\n')

    fig, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
    for field in fields.values():
        xmin,xmax,ymin,ymax=[v*sim['map_size_m']/1000 for v in field['bounds']]
        axes[0,0].add_patch(plt.Rectangle((xmin,ymin),xmax-xmin,ymax-ymin,fill=False,alpha=.35))
        axes[0,0].text((xmin+xmax)/2,(ymin+ymax)/2,field['id'],ha='center',alpha=.4)
    for name, tr in tracks.items():
        mask=np.isfinite(tr['x'])&np.isfinite(tr['y']); axes[0,0].plot(tr['x'][mask],tr['y'][mask],label=name)
        rel=tr['t']-tr['t'][0];axes[0,1].plot(rel,tr['z'],label=name);axes[0,2].plot(rel,tr['speed'],label=name)
        axes[1,0].plot(rel,tr['battery'],label=name)
    axes[0,0].set(title='Farm coverage and trajectories',xlabel='East (m)',ylabel='North (m)',aspect='equal')
    axes[0,1].set(title='Altitude',xlabel='Time (s)',ylabel='AMSL-relative height (m)')
    axes[0,2].set(title='Velocity',xlabel='Time (s)',ylabel='Speed (m/s)')
    axes[1,0].set(title='Battery remaining',xlabel='Time (s)',ylabel='Percent')
    if separation:
        st=np.array([number(r,'sim_time_s') for r in separation]);axes[1,1].scatter(st-st[0],sep_values,s=2)
        axes[1,1].axhline(sim['safety_separation_m'],color='r',linestyle='--')
    axes[1,1].set(title='Inter-UAV separation',xlabel='Simulation time (s)',ylabel='Distance (m)')
    weather = rows(experiment/'environment/weather.csv')
    wt=np.array([number(r,'sim_time_s') for r in weather]);ws=np.array([number(r,'wind_target_speed') for r in weather])
    if len(wt):axes[1,2].plot(wt-wt[0],ws)
    axes[1,2].set(title='Configured wind target',xlabel='Simulation time (s)',ylabel='Speed (m/s)')
    for axis in axes.flat:axis.grid(alpha=.2)
    axes[0,0].legend(loc='best')
    fig.savefig(analysis/'summary.png',dpi=160);plt.close(fig)
    fig,axis=plt.subplots(figsize=(7,5),constrained_layout=True)
    for drone in drones:
        data=rows(experiment/drone['name'].lower()/'telemetry.csv');mission_file=experiment/drone['name'].lower()/'mission.json'
        if not data or not mission_file.is_file():continue
        wp=[]
        for item in json.loads(mission_file.read_text()):
            if item.get('frame')==6:
                wy=math.radians(item['x']/1e7-47.397742)*6378137
                wx=math.radians(item['y']/1e7-8.545594)*6378137*math.cos(math.radians(47.397742));wp.append((wx,wy))
        if not wp:continue
        wind=[];error=[]
        for row in data:
            x,y=number(row,'x'),number(row,'y')
            if math.isfinite(x) and math.isfinite(y):
                wind.append(number(row,'wind_speed'));error.append(min(math.hypot(x-wx,y-wy) for wx,wy in wp))
        axis.scatter(wind,error,s=5,alpha=.35,label=drone['name'])
    axis.set(title='Wind versus nearest-waypoint position error',xlabel='Configured wind speed (m/s)',ylabel='Nearest waypoint distance (m)')
    axis.grid(alpha=.2);axis.legend();fig.savefig(analysis/'wind_position_error.png',dpi=160);plt.close(fig)
    print(analysis/'metrics.json')
    return metrics


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('experiment',type=Path);args=parser.parse_args();analyze(args.experiment)
