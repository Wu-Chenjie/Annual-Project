from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_ci_workflow_runs_pytest_and_cpp_build():
    workflow = ROOT / ".github" / "workflows" / "ci.yml"

    assert workflow.exists()
    source = workflow.read_text(encoding="utf-8")
    assert "python -m pytest" in source
    assert "cmake -S next_project/cpp" in source
    assert "cmake --build next_project/cpp/build_ci" in source
