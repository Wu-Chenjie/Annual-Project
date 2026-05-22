from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_ci_workflow_runs_pytest_and_cpp_build():
    workflow = ROOT / ".github" / "workflows" / "ci.yml"

    assert workflow.exists()
    source = workflow.read_text(encoding="utf-8")
    assert "python -m pytest" in source
    assert "-m \"not slow\"" in source or 'not slow' in source
    assert "working-directory: next_project" in source
    assert "cmake -S cpp -B cpp/build" in source
    assert "cmake --build cpp/build" in source
