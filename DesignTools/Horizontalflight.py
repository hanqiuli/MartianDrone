import numpy as np

class Horizontal:
   def __init__(self, b, m_init, Sf, atmos, M_MO, V_stall, Cfc=0.005, LambdaLE=0, emin=0.7, emax=0.85):
      '''
      Inputs:
         b        [m]   Wingspan                      array-like
         m_init   [kg]  Initial weight sizing         float/int
         LambdaLE [rad] Leading edge sweep            float/int
         Sf       [m^2] Fuselage wetted area          array-like
         atmos    [SI]  Atmospheric parameters        dictionary
         Cfc      [-]   Skin friction coefficient     array-like
         emin     [-]   oswald efficiency minimum     float/int
         emax     [-]   oswald efficiency maximum     float/int
      '''

      self.b         = b
      self.m         = m_init
      self.Sf        = Sf
      self.atmos     = atmos
      self.Cfc       = Cfc
      self.LambdaLE  = LambdaLE
      self.emin      = emin
      self.emax      = emax

      self.V_MO      = M_MO * self.atmos['a']


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
      if abs(LambdaLE) < 0.05:
         e = 1.78*(1-0.045*AR**0.68)-0.64

      # LE wing sweep
      else:
         e = 4.61*(1-0.045*AR**0.68)-3.1

      e = np.clip(e, self.emin, self.emax)
      
      self.K = 1/(np.pi*AR*e)


      ### Cd0
      # very crude Cd0 estimation

      SwetSref = (2*S + Sf)/S

      self.Cd0 = Cfc * SwetSref

      return self.Cd0, self.K


   def V(CL, S):
      '''
      Inputs:
         CL       [-]   Wing lift coefficient         array-like
         S        [m^2] Wing surface area             array-like
      Outputs:
         V        [m/s] True airspeed required        array-like
      '''

      W = self.atmos['g'] * self.m

      V = np.sqrt(W/S * 2/self.atmos['rho'] * 1/CL)

      return V




if __name__=="__main__":
   b = 4
   S = np.arange(2, 6, 0.1)
   LambdaLE = 0
   Sf = np.pi*0.6*0.3
   Cfc = 0.005

   atmos = {'rho':      0.017, 
            'g':        3.71, 
            'a':        233.1,
            'Re_min':   10000,
            'Re_max':   50000,
            'mu':       0.0000113}

   h = Horizontal(b, 82.23, Sf, atmos, Cfc, LambdaLE)

   print(S)
   print(h.GetAero(b, S, LambdaLE, Sf, Cfc)[0])
   print(h.GetAero(b, S, LambdaLE, Sf, Cfc)[1])