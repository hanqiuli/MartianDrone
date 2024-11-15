import sys
sys.path.append("./SETools")

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from tqdm import tqdm
from SE_data import SEData, data_path, backup_path  
from scipy.spatial import ConvexHull

data = SEData(data_path, backup_path)
mass = data.get_properties('mass')

material_props = {
    'Alloy_1': {'E': 315e9, 'v': 0.33, 'rho': 1850, 'G': 150e9, 'sigma_y': 415e6, 'embodied_energy': 6700},
    'Alloy_2': {'E': 240e9, 'v': 0.3, 'rho': 2200, 'G':99e9, 'sigma_y': 330e6, 'embodied_energy': 4600 },
    'Composite_1': {'E': 259e9, 'v': 0.2, 'rho': 2600, 'sigma_y': 249e6, 'G': 114e9,'embodied_energy': 146},
    'Composite_2': {'E': 139e9, 'v': 0.3, 'rho': 2500, 'sigma_y': 68.8e6, 'G': 55.6e9,'embodied_energy': 38.4},
}

arm_geometry = {
    "rigid_rectangle": ['width', 'height'],
    "rigid_circular": ['radius'],
    "hollow_circular": ['outer_radius', 'inner_radius'],
    "hollow_rectangle": ['outer_width', 'inner_width', 'outer_height', 'inner_height'],
    "I_beam": ['width', 'height', 'thickness_flange', 'thickness_web']
}

def convex_hull(xdata, ydata, label: str = None, color: str = 'r'):
    points = np.column_stack((xdata, ydata))
    hull = ConvexHull(points)
    for simplex in hull.simplices:
        plt.plot(points[simplex, 0], points[simplex, 1], color, lw=2,)
    plt.plot(points[hull.vertices,0], points[hull.vertices,1], color , lw=2,label=label)
    



class Arm():
    def __init__(self, material, geometry, length, **kwargs):
        self.material = material
        self.geometry = geometry
        self.length = length

        self.E = material_props[material]['E']
        self.v = material_props[material]['v']
        self.rho = material_props[material]['rho']
        self.G = material_props[material]['G']
        self.sigma_y = material_props[material]['sigma_y']
        self.embodied_energy = material_props[material]['embodied_energy']
        
        self.geometry_params = kwargs
        self.width = kwargs.get('width', None)
        self.height = kwargs.get('height', None)
        self.radius = kwargs.get('radius', None)
        self.outer_radius = kwargs.get('outer_radius', None)
        self.inner_radius = kwargs.get('inner_radius', None)
        self.outer_width = kwargs.get('outer_width', None)
        self.inner_width = kwargs.get('inner_width', None)
        self.inner_height = kwargs.get('inner_height', None)
        self.outer_height = kwargs.get('outer_height', None)
        self.thickness_flange = kwargs.get('thickness_flange', None)
        self.thickness_web = kwargs.get('thickness_web', None)

        self.Iy = self.calculate_Iy()
        self.Ix = self.calculate_Ix()
        self.A = self.calculate_A()
        self.m = self.calculate_m()
        self.j = self.calculate_j()
        self.Amean = self.calculate_Amean()
    
    def calculate_Ix(self):
        '''Calculates the moment of inertia about the x-axis (axial) for the given geometry'''
        if self.geometry == 'rigid_rectangle':
            return (1/12) * self.width * self.height**3
        elif self.geometry == 'rigid_circular':
            return (1/4) * np.pi * self.radius**4
        elif self.geometry == 'hollow_circular':
            return (1/4) * np.pi * (self.outer_radius**4 - self.inner_radius**4)
        elif self.geometry == 'hollow_rectangle':
            return (1/12) * (self.outer_width * self.outer_height**3 - self.inner_width * self.inner_height**3)
        elif self.geometry == 'I_beam':
            return (1/12) * (self.width * self.height**3 - (self.width - self.thickness_web) * (self.height - 2*self.thickness_flange)**3)
    
    def calculate_Iy(self):
        '''Calculates the moment of inertia about the y-axis (transverse) for the given geometry'''
        if self.geometry == 'rigid_rectangle':
            return (1/12) * self.height * self.width**3
        elif self.geometry == 'rigid_circular':
            return (1/4) * np.pi * self.radius**4
        elif self.geometry == 'hollow_circular':
            return (1/4) * np.pi * (self.outer_radius**4 - self.inner_radius**4)
        elif self.geometry == 'hollow_rectangle':
            return (1/12) * (self.outer_height * self.outer_width**3 - self.inner_height * self.inner_width**3)
        elif self.geometry == 'I_beam':
            return (1/12) * (self.height * self.width**3 - (self.height - 2*self.thickness_flange) * (self.width - self.thickness_web)**3)

    def calculate_A(self):
        if self.geometry == 'rigid_rectangle':
            return self.width * self.height
        elif self.geometry == 'rigid_circular':
            return np.pi * self.radius**2
        elif self.geometry == 'hollow_circular':
            return np.pi * (self.outer_radius**2 - self.inner_radius**2)
        elif self.geometry == 'hollow_rectangle':
            return self.outer_width * self.outer_height - self.inner_width * self.inner_height
        elif self.geometry == 'I_beam':
            return self.width * self.height - (self.width - self.thickness_web) * (self.height - 2*self.thickness_flange)
        
    def calculate_m(self):
        return self.A * self.length * self.rho
    
    def calculate_j(self):
        if self.geometry == 'rigid_rectangle':
            return self.width * self.height * (self.width**2 + self.height**2) / 12
        elif self.geometry == 'rigid_circular':
            return np.pi * self.radius**4 / 2
        elif self.geometry == 'hollow_circular':
            return np.pi * (self.outer_radius-self.inner_radius) * (2*self.outer_radius)**3 / 4
        elif self.geometry == 'hollow_rectangle':
            t = (self.outer_width - self.inner_width)/2
            t1 = (self.outer_height - self.inner_height)/2
            return (2*t*t1*(self.outer_width-t)**2)*(self.outer_height-t1)**2/(self.outer_width*t + self.outer_height*t1 - t**2 - t1**2)
        elif self.geometry == 'I_beam':
            return 1.3/3 *(2*self.thickness_flange**3*self.width+self.thickness_web**3*(self.height-2*self.thickness_flange))
        else:
            pass

    def calculate_Amean(self):
        if self.geometry == 'hollow_rectangle':
            return (self.inner_width + (self.inner_width+self.outer_width)/2)*( self.inner_height + (self.inner_height+self.outer_height)/2)
        else:
            pass
    
    def calculate_torsion_deflection(self, T ):
        '''Calculates the deflection of the arm under a given torque T'''
        if self.geometry == 'rigid_rectangle':
            return T * self.length / (self.G * self.j)
        elif self.geometry == 'rigid_circular':
            return T * self.length / (self.G * self.j)
        elif self.geometry == 'hollow_circular':
            return T * self.length / (self.G * self.j)
        elif self.geometry == 'hollow_rectangle':
            #return (T * self.length/(4* self.Amean**2 * self.G))*4*((self.outer_width/(self.outer_height-self.inner_height))+(self.inner_height/(self.outer_width-self.inner_width)))
            return T*self.length/(self.G*self.j)
        elif self.geometry == 'I_beam':
            #T2 = T/(1+((self.height-2*self.thickness_flange)*self.thickness_web**3)/(2*self.width*self.thickness_flange**3))
            #return 3 * T2 * self.length /(2 * self.G * self.width * self.thickness_flange**3)
            return T*self.length/(self.G*self.j)
        
    
    def calculate_deflection(self, F, axis: str = 'y', type: str = 'point'):
        '''Calculates the deflection of the arm under a given force F'''
        if axis == 'y':
            MoI = self.Iy
        elif axis == 'x':
            MoI = self.Ix
        else:
            raise ValueError('Axis must be either "x" or "y"')
        
        if type == 'point':
            return F * self.length**3 / (3 * self.E * MoI )
        elif type == 'moment':
            return F * self.length**2 / (2 * self.E * MoI)
        else:
            raise ValueError('Type must be either "point" or "moment"')
        
    def tensile_stress_bending(self,F, direction = 'y'):
        M = F*self.length #[N*m]
        if direction == 'y':
            MoI = self.Iy
        elif direction == 'x':
            MoI = self.Ix
        if self.geometry == 'rigid_rectangle':
            return M * self.height / 2*MoI
        elif self.geometry == 'rigid_circular':
            return M * self.radius / 2*MoI
        elif self.geometry == 'hollow_circular':
            return M * self.outer_radius / 2*MoI
        elif self.geometry == 'hollow_rectangle':
            return M * self.outer_height / 2*MoI
        elif self.geometry == 'I_beam':
            return M * self.height / 2*MoI
        else:
            pass

    def shear_stress(self,F):
        return F/ self.A #[Pa]
    
    def normal_stress(self,N ):
        A = self.A
        return N/A #[Pa]
    
    def buckling_critical_load(self, n):
        #determine lowest MoI:
        if self.Ix < self.Iy:
            MoI = self.Ix
        else:
            MoI = self.Iy
        return n*np.pi**2*self.E*MoI/self.length**2 #[N]
        
    


if __name__ == "__main__":
    materials = ['Alloy_1', 'Alloy_2', 'Composite_1', 'Composite_2']
    # material = 'Alloy_1'
    #length = 2.03
    length = 0.7564
    # F = 46*(1.5**2)*3.71/6
    W = 181.058 #N
    # theta = 30 * np.pi/180
    F = 81.103
    N = 40.222

    # r_blade = 1.2
    # M_torsion =3*F*r_blade/20
    # print(F)
    # print(M_torsion)
    



    # rectangulararm = Arm(material, 'rigid_rectangle', length, width=0.1, height=0.1)
    # # deflection = rectangulararm.calculate_deflection(F, axis='y', type='point')
    # rect_arm_deflection = []
    # rect_arm_mass = []
    # for i in np.arange(0.01, 0.1, 0.001):
    #     for j in np.arange(0.01, 0.1, 0.001):
    #         rectangulararm = Arm(material, 'rigid_rectangle', length, width=i, height=j)
    #         crit_buckling = rectangulararm.buckling_critical_load(0.25)
    #         bending_stress = rectangulararm.tensile_stress_bending(F, direction = 'x')
    #         normal_stress = rectangulararm.normal_stress(N)
    #         condition_1 = crit_buckling > N
    #         condition_2 = bending_stress + normal_stress < 1.25* rectangulararm.sigma_y

    #         if rectangulararm.m < 2 and condition_1 and condition_2:
    #             rect_arm_deflection.append(rectangulararm.calculate_deflection(F, axis='x', type='point'))
    #             rect_arm_mass.append(rectangulararm.m)

    # circulararm = Arm(material, 'rigid_circular', length, radius=0.1)
    # circ_arm_deflection = []
    # circ_arm_mass = []
    # for i in np.arange(0.01, 0.3, 0.001):
    #     circulararm = Arm(material, 'rigid_circular', length, radius=i)
    #     condition_1 = circulararm.buckling_critical_load(0.25) > N
    #     condition_2 = circulararm.tensile_stress_bending(F, direction = 'x') + circulararm.normal_stress(N) < 1.25* circulararm.sigma_y

    #     if circulararm.m < 2 and condition_1 and condition_2:
    #         circ_arm_deflection.append(circulararm.calculate_deflection(F, axis='x', type='point'))
    #         circ_arm_mass.append(circulararm.m)

    # hollowrectarm = Arm(material, 'hollow_rectangle', length, outer_width=0.1, inner_width=0.08, outer_height=0.1, inner_height=0.08)
    # hollowrect_arm_deflection = [] 
    # hollowrect_arm_mass = []
    # for i in np.arange(0.01, 0.1, 0.01):
    #     for j in np.arange(0.01, 0.1, 0.001):
    #         if i > j:
    #             for k in np.arange(0.01, 0.1, 0.01):
    #                 for l in np.arange(0.01, 0.1, 0.001):
    #                     if k > l:   
    #                         hollowrectarm = Arm(material, 'hollow_rectangle', length, outer_width=i, inner_width=j, outer_height=k, inner_height=l)
    #                         condition_1 = hollowrectarm.buckling_critical_load(0.25) > N
    #                         condition_2 = hollowrectarm.tensile_stress_bending(F, direction = 'x') + hollowrectarm.normal_stress(N) < 1.25* hollowrectarm.sigma_y
    #                         if hollowrectarm.m < 2 and condition_1 and condition_2:
    #                             hollowrect_arm_deflection.append(hollowrectarm.calculate_deflection(F, axis='x', type='point'))
    #                             hollowrect_arm_mass.append(hollowrectarm.m)
            
    # hollow_circle_arm = Arm(material, 'hollow_circular', length, outer_radius=0.1, inner_radius=0.08)
    # hollow_circle_arm_deflection = []
    # hollow_circle_arm_mass = []
    # for i in np.arange(0.01, 0.1, 0.001):
    #     for j in np.arange(0.01, 0.1, 0.001):
    #         if i>j:
    #             hollow_circle_arm = Arm(material, 'hollow_circular', length, outer_radius=i, inner_radius=j)
    #             condition_1 = hollow_circle_arm.buckling_critical_load(0.25) > N
    #             condition_2 = hollow_circle_arm.tensile_stress_bending(F, direction = 'x') + hollow_circle_arm.normal_stress(N) < 1.25* hollow_circle_arm.sigma_y
    #             if hollow_circle_arm.m < 2 and condition_1 and condition_2:
    #                 hollow_circle_arm_deflection.append(hollow_circle_arm.calculate_deflection(F, axis='x', type='point'))
                
    #                 hollow_circle_arm_mass.append(hollow_circle_arm.m)

    # ibeam_arm = Arm(material, 'I_beam', length, width=0.1, height=0.1, thickness_flange=0.01, thickness_web=0.01)
    # ibeam_arm_deflection = []
    # ibeam_arm_mass = []
    # for i in np.arange(0.01, 0.1, 0.005):
    #     for l in np.arange(0.001, 0.02, 0.0009):
    #         if l < i:
    #             for j in np.arange(0.01, 0.1, 0.0009):
    #                 for k in np.arange(0.001, 0.02, 0.0005):
    #                     if k<j/2:
    #                         ibeam_arm = Arm(material, 'I_beam', length, width=i, height=j, thickness_flange=k, thickness_web=l)
    #                         condition_1 = ibeam_arm.buckling_critical_load(0.25) > N
    #                         condition_2 = ibeam_arm.tensile_stress_bending(F, direction = 'x') + ibeam_arm.normal_stress(N) < 1.25* ibeam_arm.sigma_y
    #                         if ibeam_arm.m < 2 and condition_1 and condition_2:
    #                             ibeam_arm_deflection.append(ibeam_arm.calculate_deflection(F, axis='x', type='point'))
                            
    #                             ibeam_arm_mass.append(ibeam_arm.m)
  

    #use convex hull to obtain design space

    # plt.plot(circ_arm_mass, circ_arm_deflection, label='Circular Arm')
    # plt.plot(rect_arm_mass, rect_arm_deflection, label='Rectangular Arm')
    # plt.plot(hollowrect_arm_mass, hollowrect_arm_deflection, label='Hollow Rectangular Arm')
    # plt.plot(hollow_circle_arm_mass, hollow_circle_arm_deflection, label='Hollow Circular Arm')
    # plt.plot(ibeam_arm_mass, ibeam_arm_deflection, label='I-Beam Arm')
    # plt.legend()
    # plt.xlabel('Mass (kg)')
    # plt.ylabel('Deflection (m)')

    # plt.show()

    # convex_hull(circ_arm_mass, circ_arm_deflection, label='Circular Strut', color='b')
    # convex_hull(rect_arm_mass, rect_arm_deflection, label='Rectangular Strut', color='g')
    # convex_hull(hollowrect_arm_mass, hollowrect_arm_deflection, label='Hollow Rectangular Strut', color='y')
    # convex_hull(hollow_circle_arm_mass, hollow_circle_arm_deflection, label='Hollow Circular Strut', color='c')
    # convex_hull(ibeam_arm_mass, ibeam_arm_deflection, label='I-Beam Strut', color='m')
    # plt.legend()
    # plt.xlabel('Mass (kg)')
    # plt.ylabel('Deflection (m)')
    # plt.show()

    # convex_hull(circ_arm_mass, circ_arm_rotation, label='Circular Arm', color='b')
    # convex_hull(rect_arm_mass, rect_arm_rotation, label='Rectangular Arm', color='g')
    # convex_hull(hollowrect_arm_mass, hollowrect_arm_rotation, label='Hollow Rectangular Arm', color='y')
    # convex_hull(hollow_circle_arm_mass, hollow_circle_arm_rotation, label='Hollow Circular Arm', color='c')
    # convex_hull(ibeam_arm_mass, ibeam_arm_rotation, label='I-Beam Arm', color='m')

    
    # plt.legend()
    # plt.xlabel('Mass (kg)')
    # plt.ylabel('Torsion Deflection (deg)')
    # plt.show()
    
configurations_landing_strut = pd.DataFrame(columns=['outer_width', 'inner_width', 'outer_height', 'inner_height', 'mass', 'deflection','Iy', 'Ix', 'A', 'm', 'j', 'sigma_y', 'embodied_energy', 'material'])

for material in materials:
    arm = Arm(material, 'hollow_rectangle', length, outer_width=0.1, inner_width=0.08, outer_height=0.1, inner_height=0.08)

    # Define the range for the loops
    outer_width_range = np.arange(0.01, 0.040, 0.005)
    inner_width_range = np.arange(0.01, 0.040, 0.001)
    outer_height_range = np.arange(0.01, 0.040, 0.005)
    inner_height_range = np.arange(0.01, 0.040, 0.001)
    t_min = 0.001

    # Calculate the total number of iterations for the progress bar
    total_iterations = sum(1 for i in outer_width_range for j in inner_width_range if i >j +2*t_min for k in outer_height_range for l in inner_height_range if k > l + 2*t_min)

    
    # Iterate with progress bar
    with tqdm(total=total_iterations, desc="Processing Configurations") as pbar:
        for i in outer_width_range:
            for j in inner_width_range:
                if i > j+2*t_min:
                    for k in outer_height_range:
                        for l in inner_height_range:
                            if k > l+2*t_min:   
                                hollowrectarm = Arm(material, 'hollow_rectangle', length, outer_width=i, inner_width=j, outer_height=k, inner_height=l)
                                condition_1 = hollowrectarm.buckling_critical_load(0.25) > 1.25*N
                                condition_2 = hollowrectarm.tensile_stress_bending(F, direction = 'x') + hollowrectarm.normal_stress(N) < 1.25* hollowrectarm.sigma_y

                                if hollowrectarm.m < 2 and condition_1 and condition_2:
                                        new_row = pd.DataFrame({'outer_width': [i], 'inner_width': [j], 'outer_height': [k], 'inner_height': [l], 'mass': [hollowrectarm.m], 'deflection': [hollowrectarm.calculate_deflection(F, axis='x', type='point')],\
                                                                'Iy': [hollowrectarm.Iy], 'Ix': [hollowrectarm.Ix], 'A': [hollowrectarm.A], 'm': [hollowrectarm.m], 'j': [hollowrectarm.j],\
                                                                      'sigma_y': [hollowrectarm.tensile_stress_bending(F, direction = 'x')], 'embodied_energy': [hollowrectarm.embodied_energy], 'material': [material]})
                                        configurations_landing_strut = pd.concat([configurations_landing_strut, new_row], ignore_index=True) 
                                        # print(i,j,k,l)                           
                                pbar.update(1)  # Update progress bar

# Normalize the parameters
configurations_landing_strut['norm_mass'] = (configurations_landing_strut['mass'] - configurations_landing_strut['mass'].min()) / (configurations_landing_strut['mass'].max() - configurations_landing_strut['mass'].min())
configurations_landing_strut['norm_deflection'] = (configurations_landing_strut['deflection'] - configurations_landing_strut['deflection'].min()) / (configurations_landing_strut['deflection'].max() - configurations_landing_strut['deflection'].min())
# configurations_landing_strut['norm_rotation'] = (configurations_landing_strut['rotation'] - configurations_landing_strut['rotation'].min()) / (configurations_landing_strut['rotation'].max() - configurations_landing_strut['rotation'].min())
configurations_landing_strut['norm_embody'] = (configurations_landing_strut['embodied_energy'] - configurations_landing_strut['embodied_energy'].min()) / (configurations_landing_strut['embodied_energy'].max() - configurations_landing_strut['embodied_energy'].min())
# Define weights for each parameter
weight_mass = 3.0
weight_deflection = 0
weight_rotation = 0
weight_embody = 0

# Calculate the composite score
configurations_landing_strut['composite_score'] = (weight_mass * configurations_landing_strut['norm_mass'] +
                                     weight_deflection * configurations_landing_strut['norm_deflection'] +
                                     weight_embody * configurations_landing_strut['norm_embody'])

configurations_landing_strut.to_csv('configurations_landing_strut.csv', index=False)

# Find the optimal configuration
sorted_data = configurations_landing_strut.sort_values('composite_score', ascending=True)


# Print the top 5 configurations
print(sorted_data.head(50))



    
    
