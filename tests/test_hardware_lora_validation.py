import csv,tempfile,unittest
from pathlib import Path
from analysis.hardware_lora_validation import evaluate
FIELDS=("site_id","split","device_id","sf","packet_id","payload_bytes",
        "tx_timestamp_s","rx_success","tx_current_a","tx_voltage_v","tx_duration_s",
        "rx_timestamp_s","rx_rssi_dbm","rx_snr_db")
def packet(site,split,packet_id,success,sf=9):
 return dict(site_id=site,split=split,device_id="A",sf=sf,packet_id=packet_id,
    payload_bytes=12,tx_timestamp_s=packet_id*5,rx_success=int(success),
    tx_current_a=.12,tx_voltage_v=3.3,tx_duration_s=.2,
    rx_timestamp_s=(packet_id*5+.4 if success else ""),
    rx_rssi_dbm=(-118 if success else ""),rx_snr_db=(5 if success else ""))
class HardwareLogValidationTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory()
  self.addCleanup(self.tmp.cleanup)
  self.source=Path(self.tmp.name,"physical_packets.csv")
 def save(self,rows):
  with self.source.open("w",newline="") as f:
   w=csv.DictWriter(f,fieldnames=FIELDS);w.writeheader();w.writerows(rows)
 def test_site_disjoint_actual_log_is_required(self):
  self.save([packet("siteA","calibration",1,True),packet("siteA","calibration",2,False),
             packet("siteB","heldout",3,True),packet("siteB","heldout",4,False)])
  x=evaluate(self.source,Path(self.tmp.name,"report"))
  self.assertEqual(len(x),2)
  self.assertAlmostEqual(x[0]["pdr"],.5)
  self.assertAlmostEqual(x[1]["measured_tx_energy_j"],2*.12*3.3*.2)
 def test_train_test_site_leakage_rejected(self):
  self.save([packet("siteA","calibration",1,True),packet("siteA","heldout",2,True)])
  with self.assertRaises(ValueError):evaluate(self.source,Path(self.tmp.name,"report"))
 def test_success_requires_timestamps_and_signal(self):
  row=packet("a","calibration",1,True);row["rx_rssi_dbm"]=""
  self.save([row,packet("b","heldout",2,True)])
  with self.assertRaises(ValueError):evaluate(self.source,Path(self.tmp.name,"report"))
 def test_duplicate_packets_rejected(self):
  self.save([packet("a","calibration",1,True),packet("a","calibration",1,True),
             packet("b","heldout",2,True)])
  with self.assertRaises(ValueError):evaluate(self.source,Path(self.tmp.name,"report"))
if __name__=="__main__":unittest.main()
