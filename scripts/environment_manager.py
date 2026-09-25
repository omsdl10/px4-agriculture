"""Weather metadata and configured wind forcing. Dynamics handled by Gazebo WindEffects."""
import math
from common import config

def weather_at(t):
    cfg=config('simulation');base=dict(config('weather')[cfg['weather_profile']]);base.pop('moisture_scale',None)
    # This is the requested sinusoidal target, not a measured flow field.
    base['wind_target_speed']=base['wind_speed']+(base['wind_gust']-base['wind_speed'])*math.sin(2*math.pi*t/20)
    return base
