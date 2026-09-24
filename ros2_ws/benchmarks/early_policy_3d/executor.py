#!/usr/bin/env python3
"""Use the common executor, with the old centralized authorization contract."""
import view_executor_node as common
from adapter import central_execution_lease
# Same physical scan, trajectory evaluation, watchdog and separation code.
# This is explicitly centralized authorization, not peer-quorum emulation.
common.fused_execution_lease=central_execution_lease
if __name__=='__main__':common.main()
