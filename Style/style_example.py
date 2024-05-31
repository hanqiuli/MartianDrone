'''This module implements the ExampleFooBar class and the foo_bar and bar_foo functions'''

# Standard libraries
import math
import random
import sys

# Third party libraries
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

# Local imports
import FolderExample.generic_import as generic_import_named
import FolderExample.second_generic_import


def foo_bar(barb="fizzBuzz", foo_barb=None):
    '''This function does something great'''
    if foo_barb is not None:
        return barb # This function has two lines around it on both sides
    return barb.reverse()


def bar_foo(foob, bar_bar, foob_bar=True):
    '''
    This function calculates the bar-foo force

    Positional arguments:
    foob     [-]      Float       The foob coefficient
    bar_bar  [m/s]    Array-like  The bar speed

    Keyword arguments:
    foob_bar [-]      Bool        Whether or not foob is at bar

    Returns:
    barb_foo [N]      Array-like  The barb-foo force
    '''

    # The bar-foo force calculation
    barb_foo = bar_bar * (foob+foob_bar)   # [N] barb-foo force

    # note how in this case the lower level binary operator "+" has no spaces around it
    # while the higher level operator "*" does

    return barb_foo


class ExampleFooBar:
    '''Not mandatory class docstring, shortly describing this class'''

    def __init__(self, fizz_buzz):
        '''This function initializes this example class'''
        self.bizz_fuzz = fizz_buzz  # [m] The bizz fuzz distance
        self.buzz = 0               # [J] The buzz energy

    def example_instance_function(self, this, example_function, has, many, variables,
            *, in_its=None, call="signature", but="Only", five="positional arguments"):
        '''
        Many parameter function to demonstrate keyword-only arguments using "*",
        however try to avoid this many arguments
        '''

        defined = this + has + example_function + many + in_its*variables \
                    + call + but + five       # [unit] Example variable explanation
        return defined

    def second_instance_function(self, force_max):
        '''
        This function calculates the buzz energy created by force argument over bizz_fuzz distance

        Positional arguments:
        force_max   [N]     Float       The maximum force 

        Class arguments:
        bizz_fuzz   [m]     Array-like  The bizz fuzz distance

        Class outputs:
        buzz        [J]     Array-like  The buzz energy

        Returns:
        buzz        [J]     Array-like  The buzz energy
        '''

        # Note the 1 line whitespace before this part
        # Also note the 1 line before this function and the 2 lines after
        self.buzz = force_max * self.bizz_fuzz
        return self.buzz


def main():
    '''The main script'''
    # Even better to call functions and do scripting here
    foo_foo_foo = foo_bar(foo_barb=True)
    print(foo_foo_foo)

    example_object = ExampleFooBar(fizz_buzz=3)
    buzz = example_object.second_instance_function(3)

    print(buzz)

    # The code following is included to silence pylint about unused imports and show nice newlines
    sys.path.append('.')

    if "fizz" in foo_foo_foo:
        return math.sqrt(1) * random.random() + np.linspace(1, 5, 10) + sp.constants.golden_ratio

    if "zzif" in foo_foo_foo:
        plt.plot([1, 2], [2, 3])
        plt.show()
        return None

    return FolderExample.second_generic_import.foob() + generic_import_named.foob()

if __name__=="__main__":
    main()
