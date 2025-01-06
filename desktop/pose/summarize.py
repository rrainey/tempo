import sys
import re
from collections import defaultdict
from datetime import datetime, timedelta

def parse_gga_message(message):
    parts = message.split(',')
    if parts[0] == '$GPGGA' or parts[0] == '$GNGGA':
        try:
            utc_time = parts[1]  # HHMMSS.SS
            lat = float(parts[2]) / 100  # DDMM.MMMM -> DD.DDDDD
            lat_dir = parts[3]
            lon = float(parts[4]) / 100  # DDDMM.MMMM -> DDD.DDDDD
            lon_dir = parts[5]
            fix_quality = parts[6]
            satellites = int(parts[7])
            hdop = float(parts[8])
            altitude = float(parts[9])
            
            return {
                'time': utc_time,
                'latitude': f"{int(lat)}°{int((lat - int(lat)) * 60):02.3f}'{lat_dir}",
                'longitude': f"{int(lon)}°{int((lon - int(lon)) * 60):02.3f}'{lon_dir}",
                'fix_quality': fix_quality,
                'satellites': satellites,
                'hdop': hdop,
                'altitude': altitude
            }
        except ValueError:
            return None
    return None

def parse_rmc_message(message):
    parts = message.split(',')
    if parts[0] == '$GPRMC' or parts[0] == '$GNRMC':
        try:
            utc_time = parts[1]  # HHMMSS.SS
            status = parts[2]  # A=Valid, V=Invalid
            lat = float(parts[3]) / 100  # DDMM.MMMM -> DD.DDDDD
            lat_dir = parts[4]
            lon = float(parts[5]) / 100  # DDDMM.MMMM -> DDD.DDDDD
            lon_dir = parts[6]
            speed = float(parts[7]) * 1.852  # Knots to km/h
            track_angle = float(parts[8])
            date = parts[9]  # DDMMYY
            
            # Convert time and date to datetime object
            dt = datetime.strptime(f"{date}{utc_time}", "%d%m%y%H%M%S.%f")
            if status == 'V':
                return None  # Invalid data
            
            return {
                'datetime': dt,
                'latitude': f"{int(lat)}°{int((lat - int(lat)) * 60):02.3f}'{lat_dir}",
                'longitude': f"{int(lon)}°{int((lon - int(lon)) * 60):02.3f}'{lon_dir}",
                'speed': speed,
                'track_angle': track_angle
            }
        except ValueError:
            return None
    return None

def process_nmea_sentences():
    data = defaultdict(list)
    first_datetime = None
    last_datetime = None

    for line in sys.stdin:
        line = line.strip()
        gga_data = parse_gga_message(line)
        rmc_data = parse_rmc_message(line)
        
        if gga_data:
            # For GGA, we don't have the date, but we can use it for non-time data
            for key, value in gga_data.items():
                if key != 'time':
                    data[key].append(value)
            time_str = gga_data['time']
            if first_datetime:
                # Use date from the last valid RMC message to construct datetime for GGA
                dt = datetime.combine(first_datetime.date(), datetime.strptime(time_str, "%H%M%S.%f").time())
                if not last_datetime or dt > last_datetime:
                    last_datetime = dt
                if dt < first_datetime:
                    first_datetime = dt
            else:
                print("Warning: No valid RMC message found for date context. Time span might be inaccurate.")
        
        if rmc_data:
            dt = rmc_data['datetime']
            if not first_datetime or dt < first_datetime:
                first_datetime = dt
            if not last_datetime or dt > last_datetime:
                last_datetime = dt
            for key, value in rmc_data.items():
                if key != 'datetime':
                    data[key].append(value)

    # Print summary
    print("NMEA Data Summary:")
    if first_datetime and last_datetime:
        print(f"Start Timestamp: {first_datetime}")
        print(f"End Timestamp: {last_datetime}")
        print(f"Time Span: {last_datetime - first_datetime}")
    else:
        print("No valid timestamps found to calculate span.")
    
    for key, values in data.items():
        if values:  # Check if there are any values to avoid printing empty data
            print(f"{key.capitalize()}:")
            if key in ['latitude', 'longitude']:
                # For lat/lon, print first and last values
                print(f"  First: {values[0]}")
                print(f"  Last: {values[-1]}")
            else:
                print(f"  Min: {min(values)}")
                print(f"  Max: {max(values)}")
                print(f"  Average: {sum(values) / len(values)}")

if __name__ == "__main__":
    process_nmea_sentences()