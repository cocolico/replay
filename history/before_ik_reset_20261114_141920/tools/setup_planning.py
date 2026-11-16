"""Install the graph-search dependency; independent entry planning was removed."""
import argparse,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--python',default=sys.executable,help='Interpreter/environment in which to install NetworkX')
    a=p.parse_args()
    subprocess.run([a.python,'-m','pip','install','-r',str(ROOT/'requirements-planning.txt')],check=True)
