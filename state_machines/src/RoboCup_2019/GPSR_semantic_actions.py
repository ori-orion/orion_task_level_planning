#!/usr/bin/env python3
""" Code for GPSR Semantic Actions.

This file contains the class for semantic actions for the GPSR task.

Author: Charlie Street
Owner: Charlie Street

"""

class GPSRSemanticActions(object):
    """ Class for the GPSR Semantic Actions. """
    
    def s(self, ast):
        """ Return the state machine already generated. """
        return ast


    def start(self, ast):
        """ Return the state machine already generated. """
        return ast
    

    def main(self, ast):
        """ Return the state machine already generated. """
        return ast


    def pmain(self, ast):
        """ Return the result from main, ignore the polite stuff. """
        return ast[2]

    def polite(self, ast):
        return str(ast[0])


    def fndppl(self, ast):
        # TODO: findppl
        print ('findppl')
        pass
    

    def fndobj(self, ast):
        # TODO: fndobj
        print ('fndobj')
        pass

    
    def guide(self, ast):
        """ Return whatever gdcmd has generated. """
        return ast

    
    def follow(self, ast):
        # TODO: Follow
        print ('follow')
        pass
    

    def followout(self, ast):
        # TODO: followout
        print ('followout')
        pass

    
    def incomplete(self, ast):
        # TODO: incomplete
        print ('incomplete')
        pass
    

    def man(self, ast):
        """ Return whatever deliver has done. """
        return ast


    def complexman(self, ast):
        """ Return whatever cmancmd has generated. """
        return ast


    def partyhost(self, ast):
        # TODO: partyhost
        print ('partyhost')
        pass
    

    def pgenderp(self, ast):
        return str(ast[0])


    def pose(self, ast):
        return str(ast[0])


    def talk(self, ast):
        """ Return either the answer result or the speak result. """
        return ast


    def whowhere(self, ast):
        return str(ast[1]), str(ast[3])


    def findp(self, ast):
        # TODO: findp
        print ('findp')
        pass
    

    def goroom(self, ast):
        """ Just return the room. """
        return str(ast[2])


    def vbfind(self, ast):
        return str(ast[0])
    
    
    def oprop(self, ast):
        return str(ast[0])


    def gdcmd(self, ast):
        # TODO: gdcmd
        print ('gdcmd')
        pass
    

    def vbfollow(self, ast):
        return str(ast[0])


    def fllwdest(self, ast):
        """ Just return the room. """
        return str(ast[1])


    def gobeacon(self, ast):
        """ Just return the beacon to go to. """
        return str(ast[2])

    def vbguide(self, ast):
        return str(ast[0])


    def vbbring(self, ast):
        return str(ast[0])


    def vbdeliver(self, ast):
        return str(ast[0])


    def someone(self, ast):
        """ Return me, or a gesture room pair. """
        if isinstance(ast, str):
            return str(ast[0])
        else:
            return ast


    def inguidewho(self, ast):
        return str(ast[0])


    def deliver(self, ast):
        # TODO: deliver
        print ('deliver')
        pass


    def cmancmd(self, ast):
        # TODO: cmancmd
        print ('cmancmd')
        pass


    def vbserve(self, ast):
        return str(ast[0])


    def phpeopler(self, ast):
        """ Return the description of the people, and then the room. """
        return str(ast[0]), str(ast[2])


    def vbmeet(self, ast):
        return str(ast[0])
    

    def door(self, ast):
        """ Door location, space then entrance or door. """
        return str(ast[0]) + str(ast[1]) + str(ast[1])


    def taxi(self, ast):
        return str(ast[0])


    def answer(self, ast):
        """ Not really useful, so just return whole string. """
        return str(ast[0]) + str(ast[1])
    

    def speak(self, ast):
        """ Just return the whattosay part of this. """
        return str(ast[2])


    def pgenders(self, ast):
        return str(ast[0])


    def vbgopl(self, ast):
        return str(ast[0])


    def guideto(self, ast):
        """ Just return the beacon to guide to. """
        return str(ast[4])


    def gdwhere(self, ast):
        """ Just return the beacon where you can find this person """
        return str(ast[5])


    def vbbtake(self, ast):
        return str(ast[0])


    def takefrom(self, ast):
        """ object, placement returned """
        return str(ast[0]), str(ast[2])      


    def delivme(self, ast):
        """ Not very useful, just return whole string """
        return str(ast[0]) + str(ast[1])


    def delivat(self, ast):
        """ Return name, beacon """
        return str(ast[2]), str(ast[4])


    def place(self, ast):
        return str(ast[2])


    def luggage(self, ast):
        return str(ast)


    def take(self, act):
        """ Return the object. """
        return str(act[2])


    def vbplace(self, ast):
        return str(ast)


    def goplace(self, ast):
        return str(ast[2])


    def abspos(self, ast):
        """ leftmost, rightmost """
        return str(ast[0]) + str(ast[1])


    def cmanobjsrc(self, ast):
        return str(ast[1])


    def relpos(self, ast):
        if isinstance(ast, list):
            return str(ast[1])
        else:
            return str(ast)


    def vbcleanup(self, ast):
        return str(ast)


    def vbtakeout(self, ast):
        return str(ast)


    def garbage(self, ast):
        return str(ast)


    def phpeople(self, ast):
        if isinstance(ast, list):
            new_string = ""
            for string in ast:
                new_string += str(string)
            return new_string
        else:
            return str(ast)


    def vbspeak(self, ast):       
        return str(ast)


    def whattosay(self, ast):
        if isinstance(ast, list):
            new_string = ""
            for string in ast:
                new_string += str(string)
            return new_string
        else:
            return str(ast)


    def vbtake(self, ast):
        return str(ast)


    def pron(self, ast):
        return str(ast)


    def gesture(self, ast):
        return str(ast)


    def location(self, ast):
        return str(ast)


    def room(self, ast):
        return str(ast)


    def placement(self, ast):
        return str(ast)


    def beacon(self, ast):
        return str(ast)


    def name(self, ast):
        return str(ast)


    def male(self, ast):
        return str(ast)


    def female(self, ast):
        return str(ast)


    def category(self, ast):
        return str(ast)


    def object(self, ast):
        return str(ast)


    def kobject(self, ast):
        return str(ast)
    

    def aobject(self, ast):
        return str(ast)


    def sobject(self, ast):
        return str(ast)


    def question(self, ast):
        return str(ast)

    def _default(self, ast):
        raise Exception("Not Implemented")