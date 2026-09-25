from common import config

def samples():
    cfg=config('simulation');weather=config('weather')[cfg['weather_profile']];scale=cfg['map_size_m']/1000
    for sensor in config('soil')['sensors']:
        yield dict(sensor_id=sensor['id'],x=sensor['x']*scale,y=sensor['y']*scale,soil_moisture=min(100,sensor['soil_moisture']*weather.get('moisture_scale',1)),soil_temperature=sensor['soil_temperature']+(weather['temperature']-25)*.5,soil_pH=sensor['soil_pH'])
