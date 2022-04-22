import subprocess
from typing import Dict, List, Optional, Union

def scan(interface="wlan0"):
  result: List[Dict[str, Union[int, str, None]]] = []
  try:
    r = subprocess.check_output(["iwlist", interface, "scan"], encoding='utf8')

    mac:Optional[str] = None
    for line in r.split('\n'):
      if "Address" in line:
        # Based on the adapter eithere a percentage or dBm is returned
        # Add previous network in case no dBm signal level was seen
        if mac is not None:
          result.append({"mac": mac})
          mac = None

        mac = line.split(' ')[-1]
      elif "dBm" in line:
        try:
          level = line.split('Signal level=')[1]
          rss = int(level.split(' ')[0])
          result.append({"mac": mac, "rss": rss})
          mac = None
        except ValueError:
          continue

    # Add last network if no dBm was found
    if mac is not None:
      result.append({"mac": mac})

    return result

  except Exception:
    return None
