#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path


def nonempty_csv(path):
    with path.open(newline='') as stream:
        return sum(1 for _ in csv.DictReader(stream)) > 0


def validate(path):
    failures=[]
    metadata=json.loads((path/'metadata.json').read_text())
    shutdown=json.loads((path/'shutdown.json').read_text())
    if not shutdown.get('clean'):failures.append('shutdown was not clean')
    count=metadata['count']
    mission_file=path/'mission_result.json'
    if metadata['mission']!='none' and (not mission_file.is_file() or not json.loads(mission_file.read_text()).get('passed')):
        failures.append('mission did not pass')
    for i in range(1,count+1):
        uav=path/f'uav{i}'
        if not nonempty_csv(uav/'telemetry.csv'):failures.append(f'UAV{i} telemetry is empty')
        if not list((uav/'ulog').rglob('*.ulg')):failures.append(f'UAV{i} ULog is missing')
    for name in ['weather.csv','soil.csv','crop_ground_truth.csv']:
        if not nonempty_csv(path/'environment'/name):failures.append(f'{name} is empty')
    if count>1 and not nonempty_csv(path/'separation.csv'):failures.append('separation data is empty')
    recording=json.loads((path/'sensor_recording.json').read_text())
    if recording.get('errors'):failures.append('sensor recorder reported errors')
    if recording.get('dropped_frames'):failures.append('sensor recorder dropped frames')
    if metadata['sensors_enabled']:
        for i in range(1,count+1):
            for kind in ['rgb','depth']:
                if recording.get('frames',{}).get(f'UAV{i}/{kind}',0)<1:failures.append(f'UAV{i} {kind} frames missing')
    if not (path/'analysis/metrics.json').is_file():failures.append('metrics missing')
    result={'passed':not failures,'failures':failures,'experiment':str(path)}
    (path/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return not failures


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('experiment',type=Path);args=parser.parse_args()
    raise SystemExit(0 if validate(args.experiment.resolve()) else 1)
