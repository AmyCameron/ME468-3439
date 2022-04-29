#!/usr/bin/python3

class student:
    """"A class representing a student"""
    def __init__(self, n, dob):
        self.name = n
        self.dob = dob
        self.gpa = 0.0

    def get_age(self):
        return self.dob

    def set_gpa(self, g):
        self.gpa = g

    def get_gpa(self):
        return self.gpa

mary = student('Mary', (1999, 12, 30))
print("Here's Mary's DOB: ", getattr(mary,'dob'))
print("Get Mary's name: ", getattr(mary,'name'))
mary.set_gpa(4.0)
print("Get Mary's GPA: ", getattr(mary,'get_gpa')())
print("Get Mary's height: ", getattr(mary,'get_height')())


