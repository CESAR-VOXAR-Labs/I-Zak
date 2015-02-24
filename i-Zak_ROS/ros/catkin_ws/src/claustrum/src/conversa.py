import claustrum

    #answers

def do_answer_capital():
    claustrum.say("The capital of Brazil is Brazilia")

def do_answer_plus():
    claustrum.say("Two plus two is four")

def do_answer_olympics():
    claustrum.say("There are five rings in the olympic symbol")

def do_answer_robocup():
    claustrum.say("It will be in China")

def do_answer_sides():
    claustrum.say("A square has four sides")

def do_answer_christ():
    claustrum.say("Christ Redeemer is in the state of Reo de Janeiro")

def do_answer_river():
    claustrum.say("The Tea etay river is in Saum Paulo state")

def do_answer_elevator():
    claustrum.say("The La serdar elevator is in Bye. ia state")

def do_answer_university():
    claustrum.say("This university is the University of Saum Paulo")

def do_answer_robocup_league():
    claustrum.say("The name of this league is at. Home.")


def answer_questions(string):
    #if (claustrum.is_like("capital", str)):
    #    do_answer_capital() # What is the capital of Brazil?
    #elif (claustrum.is_like("brazil", str)):
    #    do_answer_capital() # What is the capital of Brazil?
    #elif (claustrum.is_like("two plus", str)):
    #    do_answer_plus() # How much is two plus two?
    #elif (claustrum.is_like("plus two", str)):
    #    do_answer_plus() # How much is two plus two?
    #elif (claustrum.is_like("how much is", str)):
    #    do_answer_plus() # How much is two plus two?
    if (claustrum.is_like("how many rings", str)):
        do_answer_olympics() # How many rings have the symbol of the Olympics?
    elif (claustrum.is_like("many rings", str)):
        do_answer_olympics() # How many rings have the symbol of the Olympics?
    elif (claustrum.is_like("symbol", str)):
        do_answer_olympics() # How many rings have the symbol of the Olympics?
    elif (claustrum.is_like("olympics", str)):
        do_answer_olympics() # How many rings have the symbol of the Olympics?
    #elif (claustrum.is_like("thousand fifteen", str)):
    #    do_answer_robocup() # Where will be the RoboCup 2015?
    #elif (claustrum.is_like("how many sides", str)):
    #    do_answer_sides() # How many sides have a square?
    #elif (claustrum.is_like("many sides", str)):
    #    do_answer_sides() # How many sides have a square?
    #elif (claustrum.is_like("square", str)):
    #    do_answer_sides() # How many sides have a square?
    #elif (claustrum.is_like("the christ", str)):
    #    do_answer_christ() # In what state is the Christ Redeemer?            
    #elif (claustrum.is_like("river", str)):
    #    do_answer_river() # In what state is the Tiete river?
    #elif (claustrum.is_like("Tiete", str)):
    #    do_answer_river() # In what state is the Tiete river?
    elif (claustrum.is_like("lacerda", str)):
        do_answer_elevator() # In what state is the Lacerda elevator?
    elif (claustrum.is_like("elevator", str)):
        do_answer_elevator() # In what state is the Lacerda elevator?
    elif (claustrum.is_like("state", str)):
        do_answer_elevator() # In what state is the Lacerda elevator?
    elif (claustrum.is_like("university", str)):
        do_answer_university() # What is the name of this University?
    elif (claustrum.is_like("this university", str)):
        do_answer_university() # What is the name of this University?
    elif (claustrum.is_like("name", str)):
        do_answer_university() # What is the name of this University?
    #elif (claustrum.is_like("robocup league", str)):
    #    do_answer_robocup_league() # What is the name of this RoboCup league?
    #elif (claustrum.is_like("this robocup", str)):
    #    do_answer_robocup_league() # What is the name of this RoboCup league?
