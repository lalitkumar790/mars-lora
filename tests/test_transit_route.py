import math,unittest
from mars_lora.transit_route import staged_route
from mars_lora.rf_astar import INDUSTRIAL,path_clearance

class StagedTransitTests(unittest.TestCase):
 def test_staged_vertical_and_planned_route(self):
  p=staged_route("shortest",INDUSTRIAL,start=(0.,0.,4.),
                 goal=(-15.,-12.,4.),transit_z=10.)
  self.assertEqual(p[0],(0.,0.,4.))
  self.assertEqual(p[1],(0.,0.,10.))
  self.assertEqual(p[-2],(-15.,-12.,10.))
  self.assertEqual(p[-1],(-15.,-12.,4.))
  self.assertGreaterEqual(path_clearance(p,radius=.32,solids=INDUSTRIAL),1.15-1e-5)
 def test_invalid_stage_rejected(self):
  for z in (float("nan"),5.):
   with self.assertRaises(ValueError):
    staged_route("shortest",INDUSTRIAL,transit_z=z)

if __name__=="__main__":
 unittest.main()
