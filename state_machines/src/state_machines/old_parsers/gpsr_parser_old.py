#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# CAVEAT UTILITOR
#
# This file was automatically generated by Grako.
#
#    https://pypi.python.org/pypi/grako/
#
# Any changes you make to it will be overwritten the next time
# the file is generated.


from __future__ import print_function, division, absolute_import, unicode_literals

from grako.buffering import Buffer
from grako.parsing import graken, Parser
from grako.util import re, RE_FLAGS, generic_main  # noqa


KEYWORDS = {}


class GPSRBuffer(Buffer):
    def __init__(
        self,
        text,
        whitespace='',
        nameguard=None,
        comments_re=None,
        eol_comments_re=None,
        ignorecase=True,
        namechars='',
        **kwargs
    ):
        super(GPSRBuffer, self).__init__(
            text,
            whitespace=whitespace,
            nameguard=nameguard,
            comments_re=comments_re,
            eol_comments_re=eol_comments_re,
            ignorecase=ignorecase,
            namechars=namechars,
            **kwargs
        )


class GPSRParser(Parser):
    def __init__(
        self,
        whitespace='',
        nameguard=None,
        comments_re=None,
        eol_comments_re=None,
        ignorecase=True,
        left_recursion=False,
        parseinfo=True,
        keywords=None,
        namechars='',
        buffer_class=GPSRBuffer,
        **kwargs
    ):
        if keywords is None:
            keywords = KEYWORDS
        super(GPSRParser, self).__init__(
            whitespace=whitespace,
            nameguard=nameguard,
            comments_re=comments_re,
            eol_comments_re=eol_comments_re,
            ignorecase=ignorecase,
            left_recursion=left_recursion,
            parseinfo=parseinfo,
            keywords=keywords,
            namechars=namechars,
            buffer_class=buffer_class,
            **kwargs
        )

    @graken()
    def _s_(self):
        self._start_()
        self._check_eof()

    @graken()
    def _start_(self):
        with self._group():
            with self._choice():
                with self._option():
                    self._pmain_()
                with self._option():
                    self._main_()
                self._error('no available options')

    @graken()
    def _pmain_(self):
        self._polite_()
        self._token(' ')
        self._main_()

    @graken()
    def _main_(self):
        self._partyhost_()

    @graken()
    def _polite_(self):
        with self._choice():
            with self._option():
                with self._group():
                    with self._choice():
                        with self._option():
                            self._token('VOID')
                        with self._option():
                            self._token('please')
                        self._error('expecting one of: VOID please')
            with self._option():
                self._token('could you')
            with self._option():
                self._token('robot please')
            with self._option():
                self._token('could you please')
            self._error('expecting one of: VOID could you could you please please robot please')

    @graken()
    def _partyhost_(self):
        with self._choice():
            with self._option():
                self._vbserve_()
                self._token(' ')
                with self._group():
                    with self._choice():
                        with self._option():
                            self._token('drinks')
                        with self._option():
                            self._token('snacks')
                        self._error('expecting one of: drinks snacks')
                self._token(' to ')
                self._phpeopler_()
            with self._option():
                self._vbmeet_()
                self._token(' ')
                self._name_()
                self._token(' at the ')
                self._door_()
                self._token(' and introduce ')
                self._pron_()
                self._token(' to ')
                self._phpeopler_()
            with self._option():
                self._vbmeet_()
                self._token(' ')
                self._name_()
                self._token(' at the ')
                self._beacon_()
                self._token(' and ask ')
                self._pron_()
                self._token(' to leave')
            with self._option():
                self._vbmeet_()
                self._token(' ')
                self._name_()
                self._token(' at the ')
                self._beacon_()
                self._token(' and introduce ')
                self._pron_()
                self._token(' to ')
                self._name_()
                self._token(' at the ')
                self._beacon_()
            with self._option():
                self._vbmeet_()
                self._token(' ')
                self._name_()
                self._token(' at the ')
                self._beacon_()
                self._token(' and ')
                self._vbguide_()
                self._token(' ')
                self._pron_()
                self._token(' to ')
                self._pron_()
                self._token(' ')
                self._taxi_()
            self._error('no available options')

    @graken()
    def _vbserve_(self):
        with self._group():
            with self._choice():
                with self._option():
                    self._token('serve')
                with self._option():
                    self._token('arrange')
                with self._option():
                    self._token('deliver')
                with self._option():
                    self._token('distribute')
                with self._option():
                    self._token('give')
                with self._option():
                    self._token('provide')
                self._error('expecting one of: arrange deliver distribute give provide serve')

    @graken()
    def _phpeopler_(self):
        self._phpeople_()
        self._token(' in the ')
        self._room_()
        self._token('.')

    @graken()
    def _vbmeet_(self):
        with self._group():
            with self._choice():
                with self._option():
                    self._token('contact')
                with self._option():
                    self._token('face')
                with self._option():
                    self._token('find')
                with self._option():
                    self._token('greet')
                self._error('expecting one of: contact face find greet')

    @graken()
    def _door_(self):
        with self._group():
            with self._choice():
                with self._option():
                    self._token('front')
                with self._option():
                    self._token('back')
                with self._option():
                    self._token('main')
                with self._option():
                    self._token('rear')
                self._error('expecting one of: back front main rear')
        self._token(' ')
        with self._group():
            with self._choice():
                with self._option():
                    self._token('entrance')
                with self._option():
                    self._token('door')
                self._error('expecting one of: door entrance')

    @graken()
    def _vbguide_(self):
        with self._group():
            with self._choice():
                with self._option():
                    self._token('guide')
                with self._option():
                    self._token('escort')
                with self._option():
                    self._token('take')
                with self._option():
                    self._token('lead')
                with self._option():
                    self._token('accompany')
                self._error('expecting one of: accompany escort guide lead take')

    @graken()
    def _taxi_(self):
        with self._group():
            with self._choice():
                with self._option():
                    self._token('taxi')
                with self._option():
                    self._token('cab')
                with self._option():
                    self._token('uber')
                self._error('expecting one of: cab taxi uber')

    @graken()
    def _phpeople_(self):
        with self._group():
            with self._choice():
                with self._option():
                    self._token('everyone')
                with self._option():
                    with self._group():
                        self._token('all the ')
                        with self._group():
                            with self._choice():
                                with self._option():
                                    self._token('people')
                                with self._option():
                                    self._token('men')
                                with self._option():
                                    self._token('women')
                                with self._option():
                                    self._token('guests')
                                with self._option():
                                    self._token('elders')
                                with self._option():
                                    self._token('children')
                                self._error('expecting one of: children elders guests men people women')
                self._error('expecting one of: all the  everyone')

    @graken()
    def _pron_(self):
        with self._choice():
            with self._option():
                self._token('it')
            with self._option():
                self._token('our')
            with self._option():
                self._token('your')
            with self._option():
                self._token('her')
            with self._option():
                self._token('its')
            with self._option():
                self._token('their')
            with self._option():
                self._token('ours')
            with self._option():
                self._token('you')
            with self._option():
                self._token('we')
            with self._option():
                self._token('his')
            with self._option():
                self._token('I')
            with self._option():
                self._token('mine')
            with self._option():
                self._token('them')
            with self._option():
                self._token('they')
            with self._option():
                self._token('hers')
            with self._option():
                self._token('him')
            with self._option():
                self._token('he')
            with self._option():
                self._token('me')
            with self._option():
                self._token('theirs')
            with self._option():
                self._token('us')
            with self._option():
                self._token('she')
            with self._option():
                self._token('my')
            with self._option():
                self._token('yours')
            self._error('expecting one of: I he her hers him his it its me mine my our ours she their theirs them they us we you your yours')

    @graken()
    def _gesture_(self):
        with self._choice():
            with self._option():
                self._token('waving')
            with self._option():
                self._token('raising their left arm')
            with self._option():
                self._token('raising their right arm')
            with self._option():
                self._token('pointing to the left')
            with self._option():
                self._token('pointing to the right')
            self._error('expecting one of: pointing to the left pointing to the right raising their left arm raising their right arm waving')

    @graken()
    def _location_(self):
        with self._choice():
            with self._option():
                self._room_()
            with self._option():
                self._placement_()
            with self._option():
                self._beacon_()
            self._error('no available options')

    @graken()
    def _room_(self):
        with self._choice():
            with self._option():
                self._token('corridor')
            with self._option():
                self._token('bedroom')
            with self._option():
                self._token('dining room')
            with self._option():
                self._token('living room')
            with self._option():
                self._token('kitchen')
            self._error('expecting one of: bedroom corridor dining room kitchen living room')

    @graken()
    def _placement_(self):
        with self._choice():
            with self._option():
                self._token('side table')
            with self._option():
                self._token('desk')
            with self._option():
                self._token('dining table')
            with self._option():
                self._token('end table')
            with self._option():
                self._token('bookcase')
            with self._option():
                self._token('cupboard')
            with self._option():
                self._token('storage table')
            with self._option():
                self._token('sink')
            with self._option():
                self._token('counter')
            self._error('expecting one of: bookcase counter cupboard desk dining table end table side table sink storage table')

    @graken()
    def _beacon_(self):
        with self._choice():
            with self._option():
                self._token('entrance')
            with self._option():
                self._token('bed')
            with self._option():
                self._token('desk')
            with self._option():
                self._token('dining table')
            with self._option():
                self._token('exit')
            with self._option():
                self._token('couch')
            with self._option():
                self._token('end table')
            with self._option():
                self._token('bookcase')
            with self._option():
                self._token('sink')
            with self._option():
                self._token('dishwasher')
            self._error('expecting one of: bed bookcase couch desk dining table dishwasher end table entrance exit sink')

    @graken()
    def _name_(self):
        with self._choice():
            with self._option():
                self._male_()
            with self._option():
                self._female_()
            self._error('no available options')

    @graken()
    def _male_(self):
        with self._choice():
            with self._option():
                self._token('alex')
            with self._option():
                self._token('charlie')
            with self._option():
                self._token('francis')
            with self._option():
                self._token('james')
            with self._option():
                self._token('john')
            with self._option():
                self._token('michael')
            with self._option():
                self._token('robert')
            with self._option():
                self._token('robin')
            with self._option():
                self._token('skyler')
            with self._option():
                self._token('william')
            self._error('expecting one of: alex charlie francis james john michael robert robin skyler william')

    @graken()
    def _female_(self):
        with self._choice():
            with self._option():
                self._token('alex')
            with self._option():
                self._token('charlie')
            with self._option():
                self._token('elizabeth')
            with self._option():
                self._token('francis')
            with self._option():
                self._token('jennifer')
            with self._option():
                self._token('linda')
            with self._option():
                self._token('mary')
            with self._option():
                self._token('patricia')
            with self._option():
                self._token('robin')
            with self._option():
                self._token('skyler')
            self._error('expecting one of: alex charlie elizabeth francis jennifer linda mary patricia robin skyler')

    @graken()
    def _category_(self):
        with self._choice():
            with self._option():
                self._token('cleaning stuff')
            with self._option():
                self._token('containers')
            with self._option():
                self._token('cutlery')
            with self._option():
                self._token('drinks')
            with self._option():
                self._token('food')
            with self._option():
                self._token('fruits')
            with self._option():
                self._token('snacks')
            with self._option():
                self._token('tableware')
            self._error('expecting one of: cleaning stuff containers cutlery drinks food fruits snacks tableware')

    @graken()
    def _object_(self):
        with self._choice():
            with self._option():
                self._kobject_()
            with self._option():
                self._aobject_()
            with self._option():
                self._sobject_()
            self._error('no available options')

    @graken()
    def _kobject_(self):
        with self._choice():
            with self._option():
                self._token('cloth')
            with self._option():
                self._token('scrubby')
            with self._option():
                self._token('sponge')
            with self._option():
                self._token('cascade pod')
            with self._option():
                self._token('fork')
            with self._option():
                self._token('knife')
            with self._option():
                self._token('spoon')
            with self._option():
                self._token('chocolate drink')
            with self._option():
                self._token('coke')
            with self._option():
                self._token('grape juice')
            with self._option():
                self._token('orange juice')
            with self._option():
                self._token('sprite')
            with self._option():
                self._token('cereal')
            with self._option():
                self._token('noodles')
            with self._option():
                self._token('sausages')
            with self._option():
                self._token('pringles')
            with self._option():
                self._token('crackers')
            with self._option():
                self._token('potato chips')
            with self._option():
                self._token('dish')
            with self._option():
                self._token('bowl')
            with self._option():
                self._token('cup')
            self._error('expecting one of: bowl cascade pod cereal chocolate drink cloth coke crackers cup dish fork grape juice knife noodles orange juice potato chips pringles sausages scrubby sponge spoon sprite')

    @graken()
    def _aobject_(self):
        with self._choice():
            with self._option():
                self._token('apple')
            with self._option():
                self._token('orange')
            with self._option():
                self._token('paprika')
            self._error('expecting one of: apple orange paprika')

    @graken()
    def _sobject_(self):
        with self._choice():
            with self._option():
                self._token('tray')
            with self._option():
                self._token('basket')
            with self._option():
                self._token('bag')
            self._error('expecting one of: bag basket tray')

    @graken()
    def _question_(self):
        self._token('question')


class GPSRSemantics(object):
    def s(self, ast):
        return ast

    def start(self, ast):
        return ast

    def pmain(self, ast):
        return ast

    def main(self, ast):
        return ast

    def polite(self, ast):
        return ast

    def partyhost(self, ast):
        return ast

    def vbserve(self, ast):
        return ast

    def phpeopler(self, ast):
        return ast

    def vbmeet(self, ast):
        return ast

    def door(self, ast):
        return ast

    def vbguide(self, ast):
        return ast

    def taxi(self, ast):
        return ast

    def phpeople(self, ast):
        return ast

    def pron(self, ast):
        return ast

    def gesture(self, ast):
        return ast

    def location(self, ast):
        return ast

    def room(self, ast):
        return ast

    def placement(self, ast):
        return ast

    def beacon(self, ast):
        return ast

    def name(self, ast):
        return ast

    def male(self, ast):
        return ast

    def female(self, ast):
        return ast

    def category(self, ast):
        return ast

    def object(self, ast):
        return ast

    def kobject(self, ast):
        return ast

    def aobject(self, ast):
        return ast

    def sobject(self, ast):
        return ast

    def question(self, ast):
        return ast


def main(filename, startrule, **kwargs):
    with open(filename) as f:
        text = f.read()
    parser = GPSRParser()
    return parser.parse(text, startrule, filename=filename, **kwargs)


if __name__ == '__main__':
    import json
    from grako.util import asjson

    ast = generic_main(main, GPSRParser, name='GPSR')
    print('AST:')
    print(ast)
    print()
    print('JSON:')
    print(json.dumps(asjson(ast), indent=2))
    print()
