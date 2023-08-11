#!/usr/bin/env python3

from keyboard import read_key
from time import time


RHYTHM_LIST = [
    {"size": 7, "truth": [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]},
    {"size": 7, "truth": [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]}
]


class RhythmMethod():
    
    def __init__(self):
        self.rhythm_id = 0
        self.load_rhythm()
        self.recorded = []
        
        self.started = False
        self.start_time = 0.0
        
    
    def load_rhythm(self):
        self.size = RHYTHM_LIST[self.rhythm_id]["size"]
        self.truth = RHYTHM_LIST[self.rhythm_id]["truth"]
        print("Successfully loaded rhythm #%d" % self.rhythm_id)
    
    
    def run(self):
        
        print("Press the 'space' key to start ... ")
        while True:
            
            entry = read_key()
            
            if entry == "space":
                if not self.started:
                    self.recorded.append(0.0)
                    self.started = True
                    self.start_time = time()
                else:
                    self.recorded.append(time() - self.start_time)
            
            if entry == "q":
                print("Thank you for participating in this exercise!")
                break
        
        self.print_record()
        self.clean_up_record()
        self.print_record()
        
        self.show_results()
    
    
    # used to remove every second entry, since key-presses are registered twice for some reason ...
    def clean_up_record(self):
        new_record = []
        print("There are %d entries in the pre-clean version" % len(self.recorded))
        # first remove duplicates
        for i in range(len(self.recorded)):
            if i % 2 == 0:
                new_record.append(self.recorded[i])
        # then reduce to the desired length
        self.recorded = new_record[0:self.size]
        print("After cleaning, there are %d entries!" % len(self.recorded))
    
            
    def print_record(self):
        print("The recorded times were %s" % self.recorded)
        
        
    def show_results(self):
        
        self.error_list = []
        self.total_error = 0
        for i in range(self.size):
            err = self.recorded[i] - self.truth[i]
            self.error_list.append(err)
            self.total_error += abs(err)
            
        print("The list of error values are %s" % self.error_list)
        print("The total magnitude of error is %.3f" % self.total_error)
        
                
            
                
                
def main():
    
    michael = RhythmMethod()
    michael.run()
    
    
if __name__ == "__main__":
    main()