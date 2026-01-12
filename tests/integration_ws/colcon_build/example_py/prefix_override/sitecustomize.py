import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/runner/work/devros/devros/tests/integration_ws/install/example_py'
