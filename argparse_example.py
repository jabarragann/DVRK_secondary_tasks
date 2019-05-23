#!/usr/bin/env python

import argparse


parser = argparse.ArgumentParser(description='DVRK secondary task data collection')
parser.add_argument('-u', '--userId' ,  help='Specify trial ID', required=True)
parser.add_argument('-t', '--trialId',  help='Specify user ID',  required=True)

args = parser.parse_args()

print(args.userId)	
print(args.trialId)