import acquire_data as ad

import imu

def acquire_measurement(duration=20,sample_duration_in_ms=1,annotation=None):
	m = ad.acquire_all(imu.imu,sample_duration_in_ms,duration)
	m2 = ad.aggregate_and_group(m)
	return {"data":m2,"annotation":annotation}

