from testClass import Test
import numpy as np
import sys
sys.path.append("./DesignTools")

from Horizontalflight import Horizontal as Hori
from environment_properties import ENV


def cmp(a, b):
   return np.all(np.isclose(a, b, rtol=1e-5, atol=1e-8))

def ql(a,b, name, cat):
   t(cmp(a,b), name, cat, a, b)
   if not cmp(a,b):
      print(name, "\t", a/b)

if __name__ == "__main__":
   test = Test()
   t = test.testlog
   h = Hori('tiltrotor', 1, 1, 1, 1)

### GetAero
   #1.1
   h.Sf        = 5
   h.Cfc       = 0.5
   h.b         = 20
   h.LambdaLE   = 0
   CD0, k      = h.GetAero(S=2)
   
   ql(CD0, 2.250000000, "1.1/CD0", "GetAero")
   ql(k, 0.002273642, "1.1/k", "GetAero")

   #1.2
   h.Sf        = 2
   h.Cfc       = 0.005
   h.b         = 6
   h.LambdaLE   = 0
   CD0, k      = h.GetAero(S=4)

   ql(CD0, 0.012500000, "1.2/CD0", "GetAero")
   ql(k, 0.045162411, "1.2/k", "GetAero")

   #1.3
   h.Sf        = 1
   h.Cfc       = 0.005
   h.b         = 2
   h.LambdaLE   = 0
   CD0, k      = h.GetAero(S=6)

   ql(CD0, 0.010833333, "1.3/CD0", "GetAero")
   ql(k, 0.561723329, "1.3/k", "GetAero")

   #1.4
   h.Sf        = 2
   h.Cfc       = 0.005
   h.b         = 4
   h.LambdaLE   = 0.523599
   CD0, k      = h.GetAero(S=2)

   ql(CD0, 0.015000000, "1.4/CD0", "GetAero")
   ql(k, 0.056841051, "1.4/k", "GetAero")

   #1.5
   h.Sf        = 1
   h.Cfc       = 2
   h.b         = 4.5
   h.LambdaLE   = 0.523599
   CD0, k      = h.GetAero(S=4)

   ql(CD0, 4.500000000, "1.5/CD0", "GetAero")
   ql(k, 0.078601161, "1.5/k", "GetAero")

   #1.6
   h.Sf        = 3
   h.Cfc       = 0.0005
   h.b         = 4
   h.LambdaLE   = 0.523599
   CD0, k      = h.GetAero(S=4)

   ql(CD0, 0.001375000, "1.6/CD0", "GetAero")
   ql(k, 0.093620555, "1.6/k", "GetAero")

### CL
   #2.1
   W = 0.01
   h.m = W / ENV['g']
   CL = h.CL(S=2, V=0.5)

   ql(CL, 2.352941176, "2.1", "CL")

   #2.2
   W = 0.01
   h.m = W / ENV['g']
   CL = h.CL(S=2, V=4)

   ql(CL, 0.036764706, "2.2", "CL")

   #2.3
   W = 0.01
   h.m = W / ENV['g']
   CL = h.CL(S=2, V=24)

   ql(CL, 0.001021242, "2.3", "CL")

   #2.4
   W = 0.01
   h.m = W / ENV['g']
   CL = h.CL(S=0.5, V=4)

   ql(CL, 0.147058824, "2.4", "CL")

   #2.5
   W = 0.01
   h.m = W / ENV['g']
   CL = h.CL(S=12, V=4)

   ql(CL, 0.006127451, "2.5", "CL")

   #2.6
   W = 0.5
   h.m = W / ENV['g']
   CL = h.CL(S=2, V=4)

   ql(CL, 1.838235294, "2.6", "CL")

   #2.7
   W = 10
   h.m = W / ENV['g']
   CL = h.CL(S=2, V=4)

   ql(CL, 36.764705882, "2.7", "CL")

### CD
   #3.1
   h.K = 0.01
   h.Cd0 = 2
   CD = h.CD(CL=0.5)

   ql(CD, 2.002500000, "3.1", "CD")

	#3.2
   h.K = 0.01
   h.Cd0 = 2
   CD = h.CD(CL=4)

   ql(CD, 2.160000000, "3.2", "CD")

   #3.3
   h.K = 0.01
   h.Cd0 = 2
   CD = h.CD(CL=24)

   ql(CD, 7.760000000, "3.3", "CD")

   #3.4
   h.K = 0.01
   h.Cd0 = 0.5
   CD = h.CD(CL=4)

   ql(CD, 0.660000000, "3.4", "CD")

   #3.5
   h.K = 0.01
   h.Cd0 = 12
   CD = h.CD(CL=4)

   ql(CD, 12.160000000, "3.5", "CD")

   #3.6
   h.K = 0.5
   h.Cd0 = 2
   CD = h.CD(CL=4)

   ql(CD, 10.000000000, "3.6", "CD")

   #3.7
   h.K = 10
   h.Cd0 = 2
   CD = h.CD(CL=4)

   ql(CD, 162.000000000, "3.7", "CD")

### V
   #4.1
   W = 2
   h.m = W / ENV['g']
   V = h.V(CL=0.01, S=0.5)

   ql(V, 216.930457819, "4.1", "V")

   #4.2
   W = 2
   h.m = W / ENV['g']
   V = h.V(CL=0.01, S=4)

   ql(V, 76.696498885, "4.2", "V")

   #4.3
   W = 2
   h.m = W / ENV['g']
   V = h.V(CL=0.01, S=24)

   ql(V, 31.311214554, "4.3", "V")

   #4.4
   W = 0.5
   h.m = W / ENV['g']
   V = h.V(CL=0.01, S=4)

   ql(V, 38.348249442, "4.4", "V")

   #4.5
   W = 12
   h.m = W / ENV['g']
   V = h.V(CL=0.01, S=4)

   ql(V, 187.867287326, "4.5", "V")

   #4.6
   W = 2
   h.m = W / ENV['g']
   V = h.V(CL=0.5, S=4)

   ql(V, 10.846522891, "4.6", "V")

   #4.7
   W = 2
   h.m = W / ENV['g']
   V = h.V(CL=10, S=4)

   ql(V, 2.425356250, "4.7", "V")

### CLrange
   #5.1
   h.V_MO = 5000
   h.K = 2
   h.Cd0 = 0.5
   W = 20
   h.m = W / ENV['g']
   CL = h.CLrange(S=0.01)

   ql(CL, 0.500000000, "5.1", "CLrange")

	#5.2
   h.V_MO = 5000
   h.K = 2
   h.Cd0 = 4
   W = 20
   h.m = W / ENV['g']
   CL = h.CLrange(S=0.01)

   ql(CL, 1.414213562, "5.2", "CLrange")

   #5.3
   h.V_MO = 5000
   h.K = 2
   h.Cd0 = 24
   W = 20
   h.m = W / ENV['g']
   CL = h.CLrange(S=0.01)

   ql(CL, 3.464101615, "5.3", "CLrange")

   #5.4
   h.V_MO = 5000
   h.K = 0.5
   h.Cd0 = 4
   W = 20
   h.m = W / ENV['g']
   CL = h.CLrange(S=0.01)

   ql(CL, 2.828427125, "5.4", "CLrange")

   #5.5
   h.V_MO = 5000
   h.K = 12
   h.Cd0 = 4
   W = 20
   h.m = W / ENV['g']
   CL = h.CLrange(S=0.01)

   ql(CL, 0.577350269, "5.5", "CLrange")

   #5.6
   h.V_MO = 50
   h.K = 2
   h.Cd0 = 4
   W = 20
   h.m = W / ENV['g']
   CL = h.CLrange(S=0.01)

   ql(CL, 94.117647059, "5.6", "CLrange")

### CLend
   #6.1
   h.V_MO = 5000
   h.K = 2
   h.Cd0 = 0.5
   W = 30
   h.m = W / ENV['g']
   CL = h.CLend(S=0.01)

   ql(CL, 0.866025404, "6.1", "CLend")

   #6.2
   h.V_MO = 5000
   h.K = 2
   h.Cd0 = 4
   W = 30
   h.m = W / ENV['g']
   CL = h.CLend(S=0.01)

   ql(CL, 2.449489743, "6.2", "CLend")

   #6.3
   h.V_MO = 5000
   h.K = 2
   h.Cd0 = 24
   W = 30
   h.m = W / ENV['g']
   CL = h.CLend(S=0.01)

   ql(CL, 6.000000000, "6.3", "CLend")

   #6.4
   h.V_MO = 5000
   h.K = 0.5
   h.Cd0 = 4
   W = 30
   h.m = W / ENV['g']
   CL = h.CLend(S=0.01)

   ql(CL, 4.898979486, "6.4", "CLend")

   #6.5
   h.V_MO = 5000
   h.K = 12
   h.Cd0 = 4
   W = 30
   h.m = W / ENV['g']
   CL = h.CLend(S=0.01)

   ql(CL, 1.000000000, "6.5", "CLend")

   #6.6
   h.V_MO = 50
   h.K = 2
   h.Cd0 = 4
   W = 30
   h.m = W / ENV['g']
   CL = h.CLend(S=0.01)

   ql(CL, 141.176470588, "6.6", "CLend")

### P_req_h
   #7.1
   P = h.P_req_h(CL_opt_r=0.001, CD_r=0.01, W=2, V_req_r=0.5)

   ql(P, 10.000000000, "7.1", "P_req_h")

   #7.2
   P = h.P_req_h(CL_opt_r=0.001, CD_r=0.01, W=2, V_req_r=4)

   ql(P, 80.000000000, "7.2", "P_req_h")

   #7.3
   P = h.P_req_h(CL_opt_r=0.001, CD_r=0.01, W=2, V_req_r=24)

   ql(P, 480.000000000, "7.3", "P_req_h")

   #7.4
   P = h.P_req_h(CL_opt_r=0.001, CD_r=0.01, W=0.5, V_req_r=4)

   ql(P, 20.000000000, "7.4", "P_req_h")

   #7.5
   P = h.P_req_h(CL_opt_r=0.001, CD_r=0.01, W=12, V_req_r=4)

   ql(P, 480.000000000, "7.5", "P_req_h")

   #7.6
   P = h.P_req_h(CL_opt_r=0.001, CD_r=0.5, W=2, V_req_r=4)

   ql(P, 4000.000000000, "7.6", "P_req_h")

   #7.7
   P = h.P_req_h(CL_opt_r=0.001, CD_r=10, W=2, V_req_r=4)

   ql(P, 80000.000000000, "7.7", "P_req_h")

   #7.8
   P = h.P_req_h(CL_opt_r=0.004, CD_r=0.1, W=2, V_req_r=4)

   ql(P, 200.000000000, "7.8", "P_req_h")

   #7.9
   P = h.P_req_h(CL_opt_r=0.3, CD_r=0.1, W=2, V_req_r=4)

   ql(P, 2.666666667, "7.9", "P_req_h")

### Getweightsys
   #9.1
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 3000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.1/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.1/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.1/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 216.331672143, "9.1/wing", "Getweightsys/wing")

   #9.2
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.2/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.2/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.2/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.2/wing", "Getweightsys/wing")

   #9.3
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 5000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.3/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.3/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.3/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 277.988798482, "9.3/wing", "Getweightsys/wing")

   #9.4
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.4/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.4/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.4/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 248.413595023, "9.4/wing", "Getweightsys/wing")

   #9.5
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.3
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.5/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.5/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.5/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 254.968325267, "9.5/wing", "Getweightsys/wing")

   #9.6
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 0.5
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.6/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.6/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.6/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 62.280942524, "9.6/wing", "Getweightsys/wing")

   #9.7
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 10
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.7/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.7/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.7/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 24912.377009774, "9.7/wing", "Getweightsys/wing")

   #9.8
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 1000
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.8/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 6121.500000000, "9.8/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.8/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.8/wing", "Getweightsys/wing")

   #9.9
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 30000
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.9/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 183645.000000000, "9.9/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.9/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.9/wing", "Getweightsys/wing")

   #9.10
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.1
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.10/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 306.075000000, "9.10/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.10/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.10/wing", "Getweightsys/wing")

   #gunky from down here
   #9.11
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.56
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.11/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1714.020000000, "9.11/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.11/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.11/wing", "Getweightsys/wing")

   #9.12
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 0.7
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.12/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 357.087500000, "9.12/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.12/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.12/wing", "Getweightsys/wing")

   #9.13
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 100
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.13/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 51012.500000000, "9.13/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.13/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.13/wing", "Getweightsys/wing")

   #9.14
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.2
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.14/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.14/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 124.437182948, "9.14/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.14/wing", "Getweightsys/wing")

   #9.15
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.95
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.15/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.15/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 26.197301673, "9.15/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.15/wing", "Getweightsys/wing")

   #9.16
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 4000
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.16/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.16/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 74795.249713480, "9.16/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 5992.738765792, "9.16/wing", "Getweightsys/wing")

   #9.17
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 3000
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.17/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.17/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 48580.939750706, "9.17/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 4970.672702390, "9.17/wing", "Getweightsys/wing")

   #9.18
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 10000
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.18/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.18/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 2313.378083367, "9.18/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.18/wing", "Getweightsys/wing")

   #9.19
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 22500
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.19/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.19/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 5205.100687576, "9.19/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.19/wing", "Getweightsys/wing")

   #9.20
   h.E_spec = 3000
   h.Range = 5500
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.20/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.20/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 48.580939751, "9.20/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.20/wing", "Getweightsys/wing")

   #9.21
   h.E_spec = 3000
   h.Range = 2000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.21/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.21/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 48.580939751, "9.21/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.21/wing", "Getweightsys/wing")

   #9.22
   h.E_spec = 0.5
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.22/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.22/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 298649.239074872, "9.22/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.22/wing", "Getweightsys/wing")

   #9.23
   h.E_spec = 300
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.005)

   ql(W_sys['const'], 84.662200000, "9.23/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.23/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 497.748731791, "9.23/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 249.123770098, "9.23/wing", "Getweightsys/wing")

   #9.24
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.002)

   ql(W_sys['const'], 84.662200000, "9.24/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.24/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 76.813210242, "9.24/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 356.132633352, "9.24/wing", "Getweightsys/wing")

   #9.25
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   W_sys = h.Getweightsys(S=0.007)

   ql(W_sys['const'], 84.662200000, "9.25/const", "Getweightsys/const")
   ql(W_sys['fuselage'], 1530.375000000, "9.25/fuselage", "Getweightsys/fuselage")
   ql(W_sys['battery'], 49.774873179, "9.25/battery", "Getweightsys/battery")
   ql(W_sys['wing'], 218.486814975, "9.25/wing", "Getweightsys/wing")

### Getweight
   #8.1
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.005, W_prop=2, W_bat_v=0.5)

   ql(W, 2092.913304828, "8.1", "Getweight")

   #8.2
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.005, W_prop=2, W_bat_v=4)

   ql(W, 2096.416804828, "8.2", "Getweight")

   #8.3
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.005, W_prop=2, W_bat_v=24)

   ql(W, 2116.436804828, "8.3", "Getweight")

   #8.4
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.005, W_prop=0.5, W_bat_v=4)

   ql(W, 2094.915304828, "8.4", "Getweight")

   #8.5
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.005, W_prop=12, W_bat_v=4)

   ql(W, 2106.426804828, "8.5", "Getweight")

   #8.6
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.002, W_prop=2, W_bat_v=4)

   ql(W, 2305.579162828, "8.6", "Getweight")

   #8.7
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.007, W_prop=2, W_bat_v=4)

   ql(W, 2044.281898295, "8.7", "Getweight")

   #8.8
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.004
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.005, W_prop=2, W_bat_v=4)

   ql(W, 2102.699772275, "8.8", "Getweight")

   #8.9
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.3
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.005, W_prop=2, W_bat_v=4)

   ql(W, 2722.619227049, "8.9", "Getweight")

   #8.10
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.1
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.005, W_prop=2, W_bat_v=4)

   ql(W, 2046.542226054, "8.10", "Getweight")

   #8.11
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.6
   h.configV['empennageWF'] = 0.4

   W = h.Getweight(S=0.005, W_prop=2, W_bat_v=4)

   ql(W, 2171.228672988, "8.11", "Getweight")

   #8.12
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.01

   W = h.Getweight(S=0.005, W_prop=2, W_bat_v=4)

   ql(W, 1999.161376219, "8.12", "Getweight")

   #8.13
   h.E_spec = 3000
   h.Range = 75000
   h.Endurance = 210
   W = 30
   h.m = W / ENV['g']
   h.eta_total = 0.5
   h.Sf = 3
   h.t_f = 0.5
   h.rho_f = 250
   h.b = 1
   h.LambdaLE = 0.1
   h.V_MO = 4000000
   h.K = 0.001
   h.Cd0 = 0.02

   h.configV['contingency'] = 0.001
   h.configV['tiltwingWF']  = 0.3
   h.configV['empennageWF'] = 0.9

   W = h.Getweight(S=0.005, W_prop=2, W_bat_v=4)

   ql(W, 2221.103251762, "8.13", "Getweight")

### Iterate_step

### Iteration

### Test Summary
   test.testsummary(True)
