import numpy as np
import matplotlib.pyplot as plt

from environment_properties import ENV
from rotor import Rotor


class Horizontal:
   def __init__(self, config, b, m_init, Sf, M_MO, V_stall, *, CL_max=1.5, w_max=4.5, update_b=True, Range=20000, Endurance=1800, Cfc=0.0065, LambdaLE=0, e_min=0.7, e_max=0.85, eta_prop=0.5, eta_power=0.6, E_spec=828000, Lf=2, Rf=0.15):
      '''
      Inputs:
         config   [-]   design configuration          string     
         b        [m]   Wingspan                      array-like
         m_init   [kg]  Initial weight sizing         float/int
         LambdaLE [rad] Leading edge sweep            float/int
         Sf       [m^2] Fuselage wetted area          array-like
         M_MO     [-]   Maximum operating mach number float/int
         V_stall  [m/s] Stall speed                   float/int
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
      self.Sf        = Sf
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


      # Derived variables
      self.V_MO      = M_MO * ENV['a']
      self.eta_total = eta_prop * eta_power
      self.WS_max    = CL_max * 0.5*ENV['rho']*V_stall**2

      if 'tiltrotor' in self.config:
         contingency = 0.2       # extra weight fraction added

         empennageWF = 0.1       # empennage weight fraction of wing mass
         tiltwingWF  = 0         # tilting mechanism weight fraction of wing mass
         tiltpropWF  = 0.25      # tilting mechanism weight fraction of propulsion mass

         winginterf  = 0.1       # Wing rotor interference  (FIND PROPER SOURCE)

         hoverTime   = 120       # [s] time spent hovering/converting to horizontal

         # rotor
         N        = 2               # Number of ROTORS
         N_blades = 4               # Number of blades per rotor

      elif 'tiltwing2' in self.config:
         contingency = 0.00       # extra weight fraction added

         empennageWF = 0.1       # empennage weight fraction of wing mass
         tiltwingWF  = 0.1       # tilting mechanism weight fraction of wing mass
         tiltpropWF  = 0.1       # tilting mechanism weight fraction of propulsion mass

         winginterf  = 0         # Wing rotor interference  (FIND PROPER SOURCE)

         hoverTime   = 300        # [s] time spent hovering/converting to horizontal

         # rotor
         N        = 2            # Number of ROTORS
         N_blades = 4            # Number of blades per rotor
      
      elif 'tiltwing4' in self.config:
         contingency = 0.00       # extra weight fraction added

         empennageWF = 0.1       # empennage weight fraction of wing mass
         tiltwingWF  = 0.1       # tilting mechanism weight fraction of wing mass
         tiltpropWF  = 0.1       # tilting mechanism weight fraction of propulsion mass

         winginterf  = 0         # Wing rotor interference  (FIND PROPER SOURCE)

         hoverTime   = 30        # [s] time spent hovering/converting to horizontal

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
      if abs(self.LambdaLE) < 0.05:
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

      CL_opt_r = np.sqrt(self.Cd0/self.K)
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

      W = (1+cfV['contingency']) * (
              Ws['wing'] * (1+cfV['tiltwingWF']+cfV['empennageWF']) 
            + Ws['fuselage'] 
            + W_prop * (1+cfV['tiltpropWF'])
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
      W_sys = {}
      
      ### Constant weights
      W_sys['const'] = 11.87 + 4.05 + 2.9 + 4      # [kg] payload, avionics, comms, other power

      ### Fuselage weight
      Pmax = 2*np.pi*self.Rf                       # [m]  fuselage perimeter
      W_sys['fuselage'] = 14.86*(self.m*ENV['g']/4.44822162)**0.144*(self.Lf/Pmax)**0.778*(self.Lf/0.3048)**(0.383)

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

      W_sys['battery'] = np.maximum(W_Bat_r, W_Bat_e)

      ### Wing weight
      Nz = 2.25   #design load factor
      taper = 1.00   #taper ratio
      AR = self.b**2 / S #Aspect ratio
      t_c_root = 0.1 #thickness/chord ratio root
      S_csw = 0.15**2 * S
      sweep_cos = np.cos(self.LambdaLE) #cos sweep=le sweep
      
      W_sys['wing'] = 0.0051 * (Nz**0.557) * S**0.649 * AR**0.5 * t_c_root**(-0.4) * (1+taper)**(-1.0) * S_csw**0.1

      return W_sys
      
   def iterate_step(self):
      ### Rotor step
      T_req_tot   = (self.m * ENV['g']) / (1-self.configV['winginterf'])

      W_rot       = self.rotor.calc_masses(T_required_tot=T_req_tot, t_flight=self.configV['hoverTime'], E_spec=self.E_spec)
      P_req_v     = self.rotor.P_total
      r_rotor     = self.rotor.r_disk

      if self.update_b:
         self.b = self.w_max - r_rotor

      ### Horizontal step
      S_low  = 1
      S_high = 6
      S_res  = 0.001
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

      
      if S_opt == S_low or S_opt == S_high:
         print(f"Optimal S is an endpoint: S = {S_opt}, indicating bad bounds")
      
      

      # Streamlines values back to floats (lazily) (also performes half iteration, source of error???)
      self.m = W_opt/ENV['g']
      self.GetAero(S_opt)
      self.Getweight(S_opt, W_rot['prop'], W_rot['battery'])
      
      P_req_h_e = self.P_req_h(self.CL_e, self.V_e, self.CD_e, W_opt)
      P_req_h_r = self.P_req_h(self.CL_r, self.V_r, self.CD_r, W_opt)

      P_req_h   = min(P_req_h_e, P_req_h_r)
      self.P_req_h_cache = P_req_h

      if P_req_h == P_req_v:
         raise NotImplementedError(f"Horizontal power required, {P_req_h}, is larger than vertical power required, {P_req_v}.\nThis case has not been covered yet")

      return S_opt, W_opt, P_req_h, P_req_v


   def iteration(self, IterMax, eps=1e-6):
      self.S_arr  = np.zeros(IterMax)
      self.W_arr  = np.zeros(IterMax)
      self.Ph_arr = np.zeros(IterMax)
      self.Pv_arr = np.zeros(IterMax)

      for i in range(IterMax):
         itVals = self.iterate_step()

         self.S_arr [i] = itVals[0]
         self.W_arr [i] = itVals[1]
         self.Ph_arr[i] = itVals[2]
         self.Pv_arr[i] = itVals[3]

      plt.plot(range(IterMax), np.transpose(np.array([self.S_arr, self.W_arr, self.Ph_arr, self.Pv_arr])))
      plt.show()
      print(itVals)




if __name__=="__main__":
   b = 4
   
   LambdaLE = 0 #sweep angle LE (rad)
   Sf = np.pi*2*0.3
   Cfc = 0.005

   hTR = Horizontal('tiltrotor', b, 82.23, Sf, 0.7 , V_stall = 100, update_b=False)
   hTW2 = Horizontal('tiltwing2', b, 82.23, Sf, 0.7 , V_stall = 110, update_b=False)
   hTW4 = Horizontal('tiltwing4', b, 82.23, Sf, 0.7 , V_stall = 110, update_b=False)

   # hTR.iteration(100)
   hTW2.iteration(100)
   hTW4.iteration(100)
