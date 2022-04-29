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

    def student_info(self):
        print('Student Name: ', self.name)
        print('Student DOB: ', self.dob) 
        print('Student GPA: ', self.gpa)



mary = student('Mary', (1999, 12, 30))
mary.set_gpa(3.9)
mary.student_info()