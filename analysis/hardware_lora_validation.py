"""Physical LoRa packet log verification: never substitute simulated data.

Accept one row per TRANSMITTED packet. Missing receive is encoded rx_success=0;
timestamps/RSSI/SNR may be blank when dropped. Site IDs must partition
calibration and held-out verification to prevent site leakage.
"""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
from collections import defaultdict
from statistics import mean
REQUIRED=("site_id","split","device_id","sf","packet_id","payload_bytes",
          "tx_timestamp_s","rx_success","tx_current_a","tx_voltage_v","tx_duration_s")
OPTIONAL=("rx_timestamp_s","rx_rssi_dbm","rx_snr_db","distance_m","obstruction_tag")
SFS=range(7,13)
def valid_float(row,key,allow_blank=False):
    raw=row.get(key,"")
    if allow_blank and (raw is None or raw==""):return None
    try:value=float(raw)
    except (ValueError,TypeError):raise ValueError(f"Bad numeric {key}: {raw!r}") from None
    if not math.isfinite(value):raise ValueError(f"Non-finite {key}")
    return value
def evaluate(path,output):
    rows=list(csv.DictReader(Path(path).open(newline="")))
    if not rows:raise ValueError("No packet measurements supplied")
    if not set(REQUIRED).issubset(rows[0]):
        raise ValueError("Required CSV fields: "+", ".join(REQUIRED))
    groups=defaultdict(list);site_split={};identifiers=set()
    for row in rows:
        site=row["site_id"].strip();split=row["split"].strip()
        if not site or split not in ("calibration","heldout"):
            raise ValueError("All packets require site and calibration/heldout split")
        if site in site_split and site_split[site]!=split:
            raise ValueError("Site leakage between calibration and heldout: "+site)
        site_split[site]=split
        sf=int(valid_float(row,"sf"))
        if sf not in SFS:raise ValueError("Invalid spreading factor")
        success=int(valid_float(row,"rx_success"))
        if success not in (0,1):raise ValueError("rx_success must be 0/1")
        key=(site,row["device_id"],row["packet_id"])
        if key in identifiers:raise ValueError("Duplicate TX packet "+str(key))
        identifiers.add(key)
        payload=valid_float(row,"payload_bytes")
        t=valid_float(row,"tx_timestamp_s")
        current=valid_float(row,"tx_current_a")
        volts=valid_float(row,"tx_voltage_v")
        toa=valid_float(row,"tx_duration_s")
        if min(payload,current,volts,toa)<0 or toa<=0:raise ValueError("Invalid physical TX properties")
        arrival=valid_float(row,"rx_timestamp_s",True)
        rssi=valid_float(row,"rx_rssi_dbm",True)
        snr=valid_float(row,"rx_snr_db",True)
        if success and (arrival is None or rssi is None or snr is None or arrival<t):
            raise ValueError("Successful packet needs RX time, RSSI and SNR")
        groups[(split,site,sf)].append(dict(success=success,rssi=rssi,
           snr=snr,delay=arrival-t if success else None,energy_j=current*volts*toa))
    if not {"calibration","heldout"}.issubset(set(site_split.values())):
        raise ValueError("Independent physical calibration AND heldout sites required")
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    report=[]
    for (split,site,sf),records in sorted(groups.items()):
        good=[r for r in records if r["success"]]
        report.append(dict(split=split,site_id=site,sf=sf,
          tx_packets=len(records),received_packets=len(good),
          pdr=len(good)/len(records),
          mean_rx_rssi_dbm=mean(r["rssi"] for r in good) if good else "",
          mean_rx_snr_db=mean(r["snr"] for r in good) if good else "",
          mean_one_way_latency_s=mean(r["delay"] for r in good) if good else "",
          measured_tx_energy_j=sum(r["energy_j"] for r in records)))
    with (out/"physical_summary.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(report[0]))
        w.writeheader();w.writerows(report)
    (out/"validation.json").write_text(json.dumps({
      "source":"User-supplied physical LoRa TX/RX logs, never simulated",
      "raw_rows":len(rows),
      "sites":site_split,
      "calibration_packets":sum(r["tx_packets"] for r in report if r["split"]=="calibration"),
      "heldout_packets":sum(r["tx_packets"] for r in report if r["split"]=="heldout"),
      "limitations":[
          "TX current*voltage*time includes neither rotor energy nor MCU/RX consumption",
          "One-way latency requires synchronized receiver and sender clocks",
          "RSSI and SNR are only defined on successfully received packets",
          "No confidence claim unless sampling covers independently randomized settings"
      ]},indent=2)+"\n")
    return report
if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--input",required=True)
    p.add_argument("--output",default="results/physical_lora_validation")
    a=p.parse_args()
    evaluate(a.input,a.output)
