#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys

from sim_cmd import get_simulator_cmd
from scripts_lib import read_test_dot_seed, subst_vars
from test_entry import get_test_entry

_CORE_IBEX = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))


def get_test_sim_cmd(base_cmd, test, binary, seed, sim_dir):
    '''Generate the command that runs a test iteration in the simulator

    base_cmd is the command to use before any test-specific substitutions. test
    is a dictionary describing the test (originally read from the testlist YAML
    file). binary is the path to the binary for the test. seed is the seed to
    use.

    sim_dir is the directory to which the test results will be written.

    Returns the command to run.

    '''
    it_cmd = subst_vars(base_cmd, {'seed': str(seed)})
    sim_cmd = (it_cmd + ' ' + test['sim_opts'].replace('\n', ' ')
               if "sim_opts" in test
               else it_cmd)

    test_name = test['test']

    # Do final interpolation into the test command for variables that depend on
    # the test name or iteration number.
    sim_cmd = subst_vars(sim_cmd,
                         {
                             'sim_dir': sim_dir,
                             'rtl_test': test['rtl_test'],
                             'binary': binary,
                             'test_name': test_name,
                         })

    if not os.path.exists(binary):
        raise RuntimeError('When computing simulation command for running '
                           'seed {} of test {}, cannot find the '
                           'expected binary at {!r}.'
                           .format(seed, test_name, binary))

    return sim_cmd


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--simulator', required=True)
    parser.add_argument("--en_cov", action='store_true')
    parser.add_argument("--en_wave", action='store_true')
    parser.add_argument('--signature-addr', required=True)
    parser.add_argument('--test-dot-seed',
                        type=read_test_dot_seed,
                        required=True)
    parser.add_argument('--binary', required=True)
    parser.add_argument('--rtl-sim-dir', required=True)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--sim-opts')

    args = parser.parse_args()

    testname, seed = args.test_dot_seed
    entry = get_test_entry(testname)

    # Look up how to run the simulator in general
    enables = {
        'cov_opts': args.en_cov,
        'wave_opts': args.en_wave
    }
    _, base_cmd = get_simulator_cmd(args.simulator, enables)

    sim_opts = f'+signature_addr={args.signature_addr}'
    if args.sim_opts:
        sim_opts += ' ' + args.sim_opts

    # Specialize base_cmd with the right directories and simulator options
    sim_cmd = subst_vars(base_cmd,
                         {
                             'out': args.rtl_sim_dir,
                             'sim_opts': sim_opts,
                             'cwd': _CORE_IBEX,
                         })

    # Specialize base_cmd for this specific test
    test_cmd = get_test_sim_cmd(sim_cmd, entry,
                                args.binary, seed, args.out_dir)

    # Run test_cmd (it's a string, so we have to call out to the shell to do
    # so). Note that we don't capture the success or failure of the subprocess:
    # if something goes horribly wrong, we assume we won't have a matching
    # trace.
    sim_log = os.path.join(args.out_dir, 'sim.log')
    os.makedirs(args.out_dir, exist_ok=True)
    with open(sim_log, 'wb') as sim_fd:
        subprocess.run(test_cmd, shell=True, stdout=sim_fd, stderr=sim_fd)

    # Always return 0 (success), even if the test failed. We've successfully
    # generated a log either way.
    return 0


if __name__ == '__main__':
    sys.exit(main())
