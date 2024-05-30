import numpy as np

class Test:
   def __init__(self):
      # Totals
      self.numTest             = 0
      self.numPassed           = 0
      self.testsFailed         = []

      # Per category
      self.categoryTest        = {}
      self.categoryPassed      = {}
      self.categoryTestsFailed = {}

   def testlog(self, passed, name, category=None, result=None, expected=None):
      if category is None:
         print(f"{name} does not have test category applied, continuing...")

      else:
         catNum = self.categoryTest.get(category, 0)
         self.categoryTest[category] = catNum+1

         catPass = self.categoryPassed.get(category, 0)

         if passed:
            catPass += 1

         self.categoryPassed[category] = catPass #this makes sure key is created



      self.numTest += 1
      if passed:
         self.numPassed += 1

         # makes sure key is created
         catLog = self.categoryTestsFailed.get(category, [])
         self.categoryTestsFailed[category] = catLog

      else:
         testlog = [name]
         if (not (result is None)) and (not (expected is None)):
            testlog.append(result)
            testlog.append(expected)

         self.testsFailed.append(testlog)

         catLog = self.categoryTestsFailed.get(category, [])
         catLog.append(testlog)
         self.categoryTestsFailed[category] = catLog
   
   def testsummary(self, skipPassedCat=True):
      with np.printoptions(precision=2, threshold=4, edgeitems=1, suppress=True):
         NameWidth = 20
         NumWidth  = 25
         Padding   = 2*" "

         linewidth = NameWidth + 2*NumWidth + 2*len(Padding)
         
         print("\n")
         print("#"*linewidth)
         print()
         print(f"{self.numPassed}/{self.numTest} tests passed")
         print()

         
         if self.numPassed == self.numTest:
            if skipPassedCat: # end summary if categories not forced printed
               print("#"*linewidth)

               return
         else:
            # Failed tests
            print(f"{'Test Name'.ljust(NameWidth)}{Padding}{'Result'.ljust(NumWidth)}{Padding}{'Expected'.ljust(NumWidth)}")

            for entry in self.testsFailed:
               entryname = str(entry[0])

               if len(entry) > 2:
                  entryresult    = entry[1]
                  entryexpected  = entry[2]
               else:
                  entryresult    = ""
                  entryexpected  = ""

               entryresult = str(entryresult)
               entryexpected = str(entryexpected)
               
               print(f"{entryname.ljust(NameWidth)}{Padding}{entryresult.ljust(NumWidth)}{Padding}{entryexpected.ljust(NumWidth)}")

         

         for category, num in self.categoryTest.items():
            passed      = self.categoryPassed[category]
            categorylog = self.categoryTestsFailed[category]

            if passed == num and skipPassedCat: # Skipping passed categories
               continue

            print("-"*linewidth)
            print()
            print(f"{category.ljust(NameWidth)}{Padding}{'-'*(linewidth-len(Padding)-NameWidth)}")

            print(f"{passed}/{num} tests passed in category {category}")

            for entry in categorylog:
               entryname = str(entry[0])

               if len(entry) > 2:
                  entryresult    = entry[1]
                  entryexpected  = entry[2]
               else:
                  entryresult    = ""
                  entryexpected  = ""

               entryresult = str(entryresult)
               entryexpected = str(entryexpected)

               print()   
               print(f"{'Test Name'.ljust(NameWidth)}{Padding}{'Result'.ljust(NumWidth)}{Padding}{'Expected'.ljust(NumWidth)}")
               print(f"{entryname.ljust(NameWidth)}{Padding}{entryresult.ljust(NumWidth)}{Padding}{entryexpected.ljust(NumWidth)}")

         print()
         print("#"*linewidth)
   