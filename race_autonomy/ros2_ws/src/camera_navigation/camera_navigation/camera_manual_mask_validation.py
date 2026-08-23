"""Offline manual-mask contract, path, control, and debug validator."""
import argparse
from pathlib import Path
import yaml
from .geometry_validation import save_report
from .manual_sample import evaluate_manual_sample

def main():
    parser=argparse.ArgumentParser();parser.add_argument("--sample",required=True);parser.add_argument("--output",required=True);parser.add_argument("--debug-output",default=None)
    args=parser.parse_args();report=evaluate_manual_sample(args.sample,args.debug_output);destination=save_report(report,args.output);print(f"{report['status']}: {destination}")
