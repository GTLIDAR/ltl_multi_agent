# -*- coding: utf-8 -*-

from os.path import abspath, dirname, join
from subprocess import check_output
from codecs import getdecoder
from argparse import ArgumentParser
from ltl_automaton_planner.ltl_tools.promela import Parser

def run_ltl2ba(formula):
    raw_output = check_output(["rosrun", "ltl2ba", "ltl2ba", "-f", "%s" % formula])
    ascii_decoder = getdecoder("ascii")
    (output, _) = ascii_decoder(raw_output)
    #print 'Output from ltl2ba'
    #print output
    return output

def parse_ltl(formula):
    ltl2ba_output = run_ltl2ba(formula)
    parser = Parser(ltl2ba_output)
    edges = parser.parse()
    return edges

if __name__ == "__main__":
    argparser = ArgumentParser(description="Call the ltl2ba program and parse the output")
    argparser.add_argument('LTL')
    args = argparser.parse_args()
    ltl2ba_output = run_ltl2ba(args.LTL)
    parser = Parser(ltl2ba_output)
    transitions = parser.parse()
    print(transitions)
