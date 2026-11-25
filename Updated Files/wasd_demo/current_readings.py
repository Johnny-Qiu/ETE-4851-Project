"""Convert Nucleo INA219 telemetry; never substitute zero for missing data."""
import math


def readings(fields, fresh, shunt_ohms=0.1):
    result = []
    for i in (1, 2):
        item = dict(address=hex(0x3f+i), status='offline', current_a=None,
                    voltage_v=None, power_w=None)
        if not fresh:
            item['status'] = 'stale'
        elif fields.get('INA') != '1':
            item['status'] = 'firmware update required'
        else:
            try:
                status = int(fields[f'ST{i}'])
                raw, mv = int(fields[f'SH{i}']), int(fields[f'BV{i}'])
                if status == 1 and -32000 < raw < 32000 and 0 <= mv <= 32000:
                    amps, volts = raw * 0.00001 / shunt_ohms, mv / 1000
                    item.update(status='ok', current_a=amps, voltage_v=volts,
                                power_w=amps*volts)
                elif status == 2:
                    item['status'] = 'measurement range exceeded'
            except (ValueError, KeyError):
                item['status'] = 'invalid reading'
        result.append(item)
    return result


def shunt_value(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Shunt resistance must be positive and finite')
    return value
