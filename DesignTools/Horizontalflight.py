import numpy as np
import matplotlib.pyplot as plt
import os

from environment_properties import ENV
from rotor import Rotor


def filepath(filename, *path):
   folders = os.path.join(*path)
   os.makedirs(folders, exist_ok=True)

   fullpath = os.path.join(folders, filename)
   return fullpath


class Horizontal:
   def __init__(self,      config,        b,             m_init,     M_MO,          V_stall,       *, 
               t_f=0.0015,  rho_f=2699,    CL_max=1.5,    w_max=4.5,  update_b=True, Range=20000,   Endurance=1800, 
               Cfc=0.005,  LambdaLE=0,    e_min=0.7,     e_max=0.85, eta_prop=0.6,  eta_power=0.7, E_spec=9e5,
               Lf=3,       Rf=0.15):
      '''
      Inputs:
         config   [-]   design configuration          string     
         b        [m]   Wingspan                      array-like
         m_init   [kg]  Initial weight sizing         float/int
         LambdaLE [rad] Leading edge sweep            float/int
         M_MO     [-]   Maximum operating mach number float/int
         V_stall  [m/s] Stall speed                   float/int
         t_f      [m]   Fuselage average thickness    float/int
         rho_f    [kg/m3]  Fuselage structure density float/int         (Aluminium)
         CL_max   [-]   Maximum lift coefficient      float/int
         w_max    [m]   Maximum width of drone        float/int
         update_b [-]   Update b w.r.t. rotor radius  bool
         Range    [m]   Range to be sized for         float/int
         Endurance[s]   Endurance to be sized for     float/int
         Cfc      [-]   Skin friction coefficient     array-like        https://archive.org/details/aircraftdesignco0000raym_y3s8/page/328/mode/2up?view=theater
         e_min    [-]   oswald efficiency minimum     float/int
         e_max    [-]   oswald efficiency maximum     float/int
         eta_prop [-]   Propeller efficiency          float/int
         eta_power[-]   Powertrain efficiency         float/int
         E_spec   [J/kg]Battery specific energy       float/int
         Lf       [m]   Fuselage length               float/int
         Rf       [m]   Fuselage radius               float/int
      '''

      # Direct assignments
      self.b         = b
      self.m         = m_init
      self.CL_max    = CL_max
      self.w_max     = w_max
      self.Cfc       = Cfc
      self.LambdaLE  = LambdaLE
      self.e_min     = e_min
      self.e_max     = e_max
      self.eta_prop  = eta_prop
      self.eta_power = eta_power
      self.E_spec    = E_spec
      self.Lf        = Lf
      self.Rf        = Rf
      self.config    = config
      self.Range     = Range
      self.Endurance = Endurance
      self.update_b  = update_b
      self.t_f       = t_f
      self.rho_f     = rho_f


      # Derived variables
      self.V_MO      = M_MO * ENV['a']
      self.eta_total = eta_prop * eta_power
      self.WS_max    = CL_max * 0.5*ENV['rho']*V_stall**2
      self.Sf        = np.pi*2*Rf*Lf*1.1

      if 'tiltrotor' in self.config:
         contingency = 0.00      # extra weight fraction added

         empennageWF = 0.1       # empennage weight fraction of wing mass
         tiltwingWF  = 0         # tilting mechanism weight fraction of wing mass
         tiltpropWF  = 0.05      # tilting mechanism weight fraction of propulsion mass

         winginterf  = 0.1       # Wing rotor interference  (FIND PROPER SOURCE)

         hoverTime   = 300       # [s] time spent hovering/converting to horizontal

         # rotor
         N        = 2               # Number of ROTORS
         N_blades = 4              # Number of blades per rotor

      elif 'tiltwing2' in self.config:
         contingency = 0.00       # extra weight fraction added

         empennageWF = 0.1       # empennage weight fraction of wing mass
         tiltwingWF  = 0.10       # tilting mechanism weight fraction of wing mass
         tiltpropWF  = 0.05       # tilting mechanism weight fraction of propulsion mass

         winginterf  = 0         # Wing rotor interference  (FIND PROPER SOURCE)

         hoverTime   = 120        # [s] time spent hovering/converting to horizontal

         # rotor
         N        = 2            # Number of ROTORS
         N_blades = 4            # Number of blades per rotor
      
      elif 'tiltwing4' in self.config:
         contingency = 0.00       # extra weight fraction added

         empennageWF = 0.1       # empennage weight fraction of wing mass
         tiltwingWF  = 0.10       # tilting mechanism weight fraction of wing mass
         tiltpropWF  = 0.05       # tilting mechanism weight fraction of propulsion mass

         winginterf  = 0         # Wing rotor interference  (FIND PROPER SOURCE)

         hoverTime   = 120        # [s] time spent hovering/converting to horizontal

         # rotor
         N        = 4            # Number of ROTORS
         N_blades = 4            # Number of blades per rotor

      else:
         raise ValueError('Selected flight configuration is invalid,\npossible configurations are: "tiltwing2", "tiltwing4", or "tiltrotor"')

      self.configV = {
         'contingency': contingency,
         'empennageWF': empennageWF,
         'tiltwingWF' : tiltwingWF ,
         'tiltpropWF' : tiltpropWF ,
         'winginterf' : winginterf ,
         'hoverTime'  : hoverTime  ,
      }

      # Rotor class
      M_tip    = 0.7             # Tip Mach number

      self.rotor = Rotor(M_tip, N, N_blades)

   def to_str(self, style="V"):
      if style not in "VI":
         raise ValueError("String style is not valid, should be (V)ertical or (I)nline")

      out_list = []
      out_vals = []
      out_str  = ""

      out_list.append("Config:")
      out_vals.append(self.config)

      out_list.append("Drone mass:")
      out_vals.append(self.m)

      out_list.append("System mass:")
      out_vals.append(self.W_sys)

      out_list.append("Powers required for vertical flight:")
      out_vals.append(self.P_req_v_cache)

      out_list.append("Powers required for horizontal flight:")
      out_vals.append(self.P_req_h_cache)

      out_list.append("V_range:")
      out_vals.append(self.V_r)

      out_list.append("r_disk:")
      out_vals.append(self.rotor.r_disk)

      out_list.append("S:")
      out_vals.append(self.S_opt)

      out_list.append("V_stall:")
      out_vals.append(self.V(self.CL_max, self.S_opt))

      largestlen = 0
      for i in range(len(out_list)):
         largestlen = max(len(out_list[i]), largestlen)

         out_val     = out_vals[i]
         out_vals[i] = ""
         if isinstance(out_val, np.ndarray):
            out_vals[i] += np.array2string(out_val.flatten()[0]) + "\n"
         elif isinstance(out_val, float) or isinstance(out_val, int):
            out_vals[i] += str(out_val) + "\n"
         elif isinstance(out_val, str):
            out_vals[i] += out_val + "\n"
         elif isinstance(out_val, dict):
            for key, val in out_val.items():
               out_vals[i] += f"{key} \t {val}\n"
         else:
            raise TypeError(f"Printed value {out_val} of type {type(out_val)} is not printable currently.")

      for _ in range(len(out_list)):
         out_str += out_list.pop(0) + "\n"
         out_val  = out_vals.pop(0)
         out_str += out_val + "\n"
            
      return out_str

   def print(self, style="V"):
      print(self.to_str(style))    

   def GetAero(self, S):
      '''
      Inputs:
         S        [m^2] Wing surface area             array-like
      Outputs:
         Cd0      [-]   Zero lift drag coefficient    array-like
         K        [-]   Lift induced drag coefficient array-like
      '''

      ### K
      AR = self.b**2/S

      # No LE wing sweep
      if abs(self.LambdaLE) < 0.5:
         e = 1.78*(1-0.045*AR**0.68)-0.64

      # LE wing sweep
      else:
         e = 4.61*(1-0.045*AR**0.68)-3.1

      e = np.clip(e, self.e_min, self.e_max)
      
      self.K = 1/(np.pi*AR*e)


      ### Cd0
      # very crude Cd0 estimation

      SwetSref = (2*S + self.Sf)/S

      self.Cd0 = self.Cfc * SwetSref

      return self.Cd0, self.K

   def CL(self, V, S):
      '''
      Inputs:
         V        [m/s] True airspeed required        array-like
         S        [m^2] Wing surface area             array-like
      Outputs:
         CL       [-]   Wing lift coefficient         array-like
      '''
      W = self.m * ENV['g']

      CL = W/(0.5*ENV['rho']*V**2 * S)

      return CL

   def CD(self, CL):
      '''
      Inputs:
         CL       [-]   Wing lift coefficient         array-like
      Outputs:
         CD       [-]   Total drag coefficient        array-like
      '''
      return self.Cd0 + self.K*CL**2

   def V(self, CL, S):
      '''
      Inputs:
         CL       [-]   Wing lift coefficient         array-like
         S        [m^2] Wing surface area             array-like
      Outputs:
         V        [m/s] True airspeed required        array-like
      '''

      W = ENV['g'] * self.m

      V = np.sqrt(W/S * 2/ENV['rho'] * 1/CL)

      return V
   
   def CLrange(self, S):
      '''
         Inputs:
         S        [m^2] Wing surface area             array-like
      Outputs:
         CL_opt_r [-]   Wing lift coefficient         array-like
      '''
      CL_min = self.CL(self.V_MO, S)

      # Optimal CL for maximum range is where (C_d / C_l) is minimized, which is sqrt(Cd0 / K)
      CL_opt_r = np.sqrt(self.Cd0 / self.K)
      CL_opt_r = np.maximum(CL_opt_r, CL_min)

      return CL_opt_r

   def CLend(self, S):
      '''
      Inputs:
         S        [m^2] Wing surface area             array-like
      Outputs:
         CL_opt_e [-]   Wing lift coefficient         array-like
      '''
      CL_min = self.CL(self.V_MO, S)

      CL_opt_e = np.sqrt(12*self.K*self.Cd0)/(2*self.K)
      CL_opt_e = np.maximum(CL_opt_e, CL_min)
      return CL_opt_e

   def P_req_h(self, CL_opt_r, V_req_r, Cd_r, W):
      '''
      Inputs:
         CL       [-]   Optimal lift coefficient for                    array-like
         V_req    [m/s] Required true airspeed                          array-like
         Cd       [-]   Drag coefficient                                array-like
         W        [N]   Weight                                          array-like
      Outputs:
         P_req    [W]   Required power for horizontal flight            array-like
      '''

      
      return (Cd_r / CL_opt_r * W * V_req_r)
   
   def Getweight(self, S, W_prop, W_bat_v):
      '''
      Inputs:
         S        [m^2] Wing surface area             array-like
         W_prop   [N]   Weight of propulsion system   array-like
         W_bat
      Outputs:
         W_tot    [N]   Total weight of iteration     array-like
      '''

      cfV = self.configV
      Ws = self.Getweightsys(S)

      Ws['wing'] *= 1+cfV['tiltwingWF']+cfV['empennageWF']

      W = (1+cfV['contingency']) * (
              Ws['wing'] 
            + Ws['fuselage'] 
            + W_prop
            + Ws['battery'] + W_bat_v
            + Ws['const']
      )

      self.m = W/ENV['g']

      return W

   def Getweightsys(self, S):
      '''
      Inputs:
         S        [m^2] Wing surface area             array-like
      Outputs:
         W_sys    [N]   System weights of iteration     dict (str -> array-likes)
      '''

      # Current weight
      W = self.m*ENV['g']
      self.W_sys = {}
      
      ### Constant weights
      self.W_sys['const'] = (11.87 + 4.05 + 2.9 + 4)*ENV['g']      # [kg] payload, avionics, comms, other power

      ### Fuselage weight
      #Pmax = 2*np.pi*self.Rf                       # [m]  fuselage perimeter
      #self.W_sys['fuselage'] = 14.86*(self.m*ENV['g']/4.44822162)**0.144*(self.Lf/Pmax)**0.778*(self.Lf/0.3048)**(0.383)

      self.W_sys['fuselage'] = ENV['g']* self.Sf*self.t_f*self.rho_f*1.1

      ### Battery weight
      # Range
      self.CL_r = self.CLrange(S)
      self.CD_r = self.CD(self.CL_r)
      self.V_r  = self.V(self.CL_r, S)
      W_Bat_r = self.Range      *ENV['g']/self.E_spec * self.CD_r/self.CL_r * W/self.eta_total

      # Endurance
      self.CL_e = self.CLend(S)
      self.CD_e = self.CD(self.CL_e)
      self.V_e  = self.V(self.CL_e, S)
      W_Bat_e = self.Endurance  *self.CD_e/self.CL_e * W * self.V_e / self.eta_total / self.E_spec * ENV['g']

      self.W_sys['battery'] = np.maximum(W_Bat_r, W_Bat_e)

      ### Wing weight
      Nz = 2.25   #design load factor
      taper = 1.00   #taper ratio
      AR = self.b**2 / S #Aspect ratio
      t_c_root = 0.1 #thickness/chord ratio root
      S_csw = 0.15**2 * S
      sweep_cos = np.cos(self.LambdaLE) #cos sweep=le sweep
      
      # self.W_sys['wing'] = ENV['g'] * (0.0051 * (Nz**0.557) * S**0.649 * AR**0.5 * t_c_root**(-0.4) * (1+taper)**(-1.0) * S_csw**0.1)  

      # taken from paper. 2.25 is assumed ultimate load factor
      M_wing = (
        96.948 *
        (self.m * Nz / 10**5)**0.65 *
        (AR / (sweep_cos**0.57)) *
        (S / 100)**0.61 *
        (1 + taper / 2 / t_c_root)**0.36 *
        (1 + (self.V_MO / 500)**0.5)**0.993
      )

      self.W_sys['wing'] = ENV['g'] * M_wing 

      return self.W_sys

   def iterate_step(self):
      ### Rotor step
      T_req_tot      = (self.m * ENV['g']) / (1-self.configV['winginterf'])

      W_rot          = self.rotor.calc_masses(T_required_tot=T_req_tot, t_flight=self.configV['hoverTime'], E_spec=self.E_spec/3600)
      W_rot['prop'] *= (1+self.configV['tiltpropWF'])

      P_req_v        = self.rotor.P_total
      r_rotor        = self.rotor.r_disk

      if self.update_b:
         self.b = self.w_max - r_rotor

      ### Horizontal step
      S_low  = 0.3
      S_high = 2.5*self.b
      S_res  = 0.0001
      S = np.arange(S_low, (S_high+S_res), S_res)

      self.GetAero(S)

      W = self.Getweight(S, W_rot['prop'], W_rot['battery'])

      ValidStall = (W/S < self.WS_max)
      MaskedW = W[ValidStall]

      if np.all(~ValidStall):
         raise Exception(f"Wing loading of {self.WS_max} could not be met within S of {S_low} and {S_high},\n Wing loadings are:\n\
         {W/S}\n\
         Power required (vert):\n\
         {P_req_v}\n\
         Power required (hor, last):\n\
         {self.P_req_h_cache}")

      i_opt = ValidStall.nonzero()[0][np.argmin(MaskedW)] # converts back from masked index to global index
      S_opt = S[i_opt]
      W_opt = W[i_opt]

      
      if S_opt < S_low+S_res/2 or S_opt > S_high-S_res/2:
         print(f"Optimal S is an endpoint: S = {S_opt}, indicating bad bounds or divergence")


      # Streamlines values back to floats (lazily) (also performes partial iteration, source of error???)
      self.m = W_opt/ENV['g']
      self.GetAero(S_opt)
      self.Getweight(S_opt, W_rot['prop'], W_rot['battery'])

      for key, val in W_rot.items():
         if key in self.W_sys.keys():
            self.W_sys[key] += val
            continue
         self.W_sys[key] = val
      
      P_req_h_e = self.P_req_h(self.CL_e, self.V_e, self.CD_e, W_opt)
      P_req_h_r = self.P_req_h(self.CL_r, self.V_r, self.CD_r, W_opt)

      P_req_h   = min(P_req_h_e, P_req_h_r)/self.eta_total
      self.P_req_v_cache = P_req_v
      self.P_req_h_cache = P_req_h
      self.S_opt = S_opt

      if P_req_h > P_req_v:
         raise NotImplementedError(f"Horizontal power required, {P_req_h}, is larger than vertical power required, {P_req_v}.\nThis case has not been covered yet")

      V_opt_range = self.V(self.CLrange(S_opt), S_opt) # Calculate the optimal range velocity
      return S_opt, W_opt, P_req_h, P_req_v, V_opt_range

   def iteration(self, IterMax, eps=1e-6, plotSave=True, plotShow=False):
      self.S_arr  = np.zeros(IterMax)
      self.W_arr  = np.zeros(IterMax)
      self.Ph_arr = np.zeros(IterMax)
      self.Pv_arr = np.zeros(IterMax)
      self.Vr_arr = np.zeros(IterMax)

      for i in range(IterMax):
         itVals = self.iterate_step()

         self.S_arr [i] = itVals[0]
         self.W_arr [i] = itVals[1]
         self.Ph_arr[i] = itVals[2]
         self.Pv_arr[i] = itVals[3]
         self.Vr_arr[i] = itVals[4]

         if (abs(self.S_arr [i-1] - itVals[0])<eps and
             abs(self.W_arr [i-1] - itVals[1])<eps and
             abs(self.Ph_arr[i-1] - itVals[2])<eps and
             abs(self.Pv_arr[i-1] - itVals[3])<eps and
             abs(self.Vr_arr[i-1] - itVals[4])<eps):

            print(f"Iter stopped at {i}")
            break
      
      if plotSave or plotShow:
         plt.plot(range(i), np.transpose(np.array([self.S_arr[:i], self.W_arr[:i]/ENV['g'], self.Ph_arr[:i]/1000, self.Pv_arr[:i]/1000, self.Vr_arr[:i]])))
         plt.legend(["S [m]", "m [kg]", "P_h [kW]", "P_v [kW]", "V_range [m/s]"])
      if plotSave:
         path = filepath(f"Iterplot for {self.config}", "plots")
         plt.savefig(path)
      if plotShow:
         plt.show()
      plt.clf()

      for key, val in self.W_sys.items():
         self.W_sys[key] = round(val, 3)





if __name__=="__main__":
   b = 6
   
   LambdaLE = 0 #sweep angle LE (rad)
   Sf = np.pi*2*0.3
   Cfc = 0.005

   hTR  = Horizontal('tiltrotor', b, 82.23, 0.7 , V_stall = 110, update_b=False, Endurance = 1200)
   hTW2 = Horizontal('tiltwing2', b, 82.23, 0.7 , V_stall = 110, update_b=False, Endurance = 1200)
   hTW4 = Horizontal('tiltwing4', b, 82.23, 0.7 , V_stall = 110, update_b=False, Endurance = 1200)

   hTR .iteration(100, plotShow=False)
   hTW2.iteration(100, plotShow=False)
   hTW4.iteration(100, plotShow=False)

   hTR.print()
   hTW2.print()
   hTW4.print()


   if False:
      print("Tiltrotor, Tiltwing2, Tiltwing4:")

      print("Drone mass:")
      print(hTR.m)
      print(hTW2.m)
      print(hTW4.m)

      print("System mass:")
      print(hTR.W_sys)
      print(hTW2.W_sys)
      print(hTW4.W_sys)

      print("Powers required for vertical flight:")
      print(hTR.P_req_v_cache)
      print(hTW2.P_req_v_cache)
      print(hTW4.P_req_v_cache)

      print("Powers required for horizontal flight:")
      print(hTR.P_req_h_cache)
      print(hTW2.P_req_h_cache)
      print(hTW4.P_req_h_cache)

      print("V_range:")
      print(hTR.V_r)
      print(hTW2.V_r)
      print(hTW4.V_r)

      print("r_disk:")
      print(hTR.rotor.r_disk)
      print(hTW2.rotor.r_disk)
      print(hTW4.rotor.r_disk)

      print("S:")
      print(hTR.S_opt)
      print(hTW2.S_opt)
      print(hTW4.S_opt)

      print("V_stall:")
      print(hTR.V(hTR.CL_max, hTR.S_opt))
      print(hTW2.V(hTW2.CL_max, hTW2.S_opt))
      print(hTW4.V(hTW4.CL_max, hTW4.S_opt))

