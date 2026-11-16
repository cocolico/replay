"""Per-joint Wuji current protection checks."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from read_hands import check_current_limit


class CurrentTests(unittest.TestCase):
    def states(self,value=0.):return {s:{'current_a':[value]*20} for s in ('left','right')}

    def test_each_joint_is_checked_without_joint_summing(self):
        states=self.states(.1)
        # Sum is 2 A, but every individual motor is below 1.5 A.
        report=check_current_limit(states,1.5)
        self.assertAlmostEqual(report['left']['maximum_abs_current_a'],.1)

    def test_absolute_current_over_one_point_five_amps_blocks(self):
        for value in (1.5001,-1.5001):
            states=self.states();states['right']['current_a'][7]=value
            with self.assertRaisesRegex(RuntimeError,'right joint 7'):
                check_current_limit(states,1.5)

    def test_incomplete_current_feedback_blocks(self):
        with self.assertRaisesRegex(ValueError,'current unavailable'):
            check_current_limit({'left':{},'right':{}},1.5)


if __name__=='__main__':unittest.main()
