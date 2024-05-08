import numpy as np
from environment_properties import ENV

class Horizontal:
   def __init__(self, b, m_init, Sf, M_MO, V_stall, Cfc=0.005, LambdaLE=0, e_min=0.7, e_max=0.85, eta_prop=0.5, eta_power=0.6, E_spec=828000):
      '''
      Inputs:
         b        [m]   Wingspan                      array-like
         m_init   [kg]  Initial weight sizing         float/int
         LambdaLE [rad] Leading edge sweep            float/int
         Sf       [m^2] Fuselage wetted area          array-like
         M_MO     [-]   Maximum operating mach number float/int
         V_stall  [m/s] Stall speed                   float/int
         Cfc      [-]   Skin friction coefficient     array-like
         e_min    [-]   oswald efficiency minimum     float/int
         e_max    [-]   oswald efficiency maximum     float/int
         eta_prop [-]   Propeller efficiency          float/int
         eta_power[-]   Powertrain efficiency         float/int
         E_spec   [J/kg]Battery specific energy       float/int
      '''

      # Direct assignments
      self.b         = b
      self.m         = m_init
      self.Sf        = Sf
      self.Cfc       = Cfc
      self.LambdaLE  = LambdaLE
      self.e_min     = e_min
      self.e_max     = e_max
      self.eta_prop  = eta_prop
      self.eta_power = eta_power
      self.E_spec    = E_spec

      # Derived variables
      self.V_MO      = M_MO * ENV['a']
      self.eta_total = eta_prop * eta_power
      

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


   # Define power functions
   def P_r_prop(self, CL_opt_r, V_req_r, Cd_r):
      '''
      Inputs:
         CL_opt_r [-]   Optimal lift coefficient for optimal range      array-like
         V_req_r  [m/s] Required true airspeed for optimal range        array-like
         Cd_r     [-]   Drag coefficient                                array-like
      '''

      W = ENV['g'] * self.m
      
      return (Cd_r / CL_opt_r * W * V_req_r)
   
   def Getweight(self, S, W_Power):
      # Current weight
      W = self.m*ENV['g']

      ### Wing weight
      Nz = 2.25   #design load factor
      taper = 1.00   #taper ratio
      AR = self.b**2/S #Aspect ratio
      t_c_root = 0.1 #thickness/chord ratio root
      S_csw = 0.15**2*S
      sweep_cos = np.cos(self.LambdaLE) #cos sweep=le sweep
      W_Wing = 0.0051 * (Nz**0.557) * S**0.649 * AR**0.5 * t_c_root**(-0.4) * (1+taper)**(-1.0) * S_csw**0.1

      constWeights = 11.87 + 4.05 + 2.9 + 4 # [kg] payload, avionics, comms, other power
      lf = 2 #fuselage length [m]
      Pmax = np.pi*0.3 #fuselage diameter [m]
      
      Wf = 14.86*(self.m*ENV['g']/4.44822162)**0.144*(lf/Pmax)**0.778*(lf/0.3048)**(0.383)


      ### Battery weight
      # Range
      CL_r = self.CLrange(S)
      CD_r = self.CD(CL_r)
      W_Bat_r = 20000   *ENV['g']/self.E_spec * CD_r/CL_r * W/self.eta_total

      # Endurance
      CL_e = self.CLend(S)
      CD_e = self.CD(CL_e)
      W_Bat_e = 1800    *CD_e/CL_e * W * self.V(CL_e, S) / self.eta_total * ENV['g']

      print(W_Bat_e)
      print(W_Bat_r)


      Wnew = W_Wing + Wf + W_Power + constWeights



if __name__=="__main__":
   b = 4
   S = np.arange(2, 6, 0.1)
   LambdaLE = 0 #sweep angle LE (rad)
   Sf = np.pi*2*0.3
   Cfc = 0.005
   
   atmos = {'rho':      0.017, 
            'g':        3.71, 
            'a':        233.1,
            'Re_min':   10000,
            'Re_max':   50000,
            'mu':       0.0000113}

   h = Horizontal(b, 82.23, Sf, 0.7 , V_stall = 15)

   print(S)
   print(h.Getweight(S, 30))