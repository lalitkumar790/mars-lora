import math,unittest
from analysis.flight_energy_proxy import hover_proxy
class HoverEnergyProxyTests(unittest.TestCase):
 def test_positive_and_efficiency(self):
  p=hover_proxy(1.325,.11,1)
  self.assertGreater(p,0)
  self.assertAlmostEqual(hover_proxy(1.325,.11,.55),p/.55,places=6)
 def test_radius_reduces_power(self):
  self.assertLess(hover_proxy(1.325,.13),hover_proxy(1.325,.10))
 def test_mass_increases_power(self):
  self.assertGreater(hover_proxy(1.325,.1),hover_proxy(.027,.1))
 def test_bad_parameters(self):
  with self.assertRaises(ValueError):hover_proxy(1.,0.)
  with self.assertRaises(ValueError):hover_proxy(1.,.1,1.2)
if __name__=="__main__":unittest.main()
