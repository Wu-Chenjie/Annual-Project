from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path

from experiments.report_cpp_results import generate_cpp_result_report


SIM_RESULT_RE = re.compile(
    r"(?P<path>(?:[A-Za-z]:[\\/][^\r\n]*?outputs[\\/][^\r\n]*?sim_result\.json|"
    r"outputs[\\/][^\r\n]*?sim_result\.json))"
)


def find_sim_result_path(stdout: str, cwd: str | Path) -> Path:
    cwd_path = Path(cwd)
    candidates: list[Path] = []
    for match in SIM_RESULT_RE.finditer(stdout):
        raw = match.group("path").strip().strip('"').strip("'")
        path = Path(raw)
        if not path.is_absolute():
            path = cwd_path / path
        candidates.append(path)
    for path in candidates:
        if path.is_file():
            return path.resolve()
    if candidates:
        return candidates[-1].resolve()
    raise FileNotFoundError("could not find outputs/.../sim_result.json in C++ stdout")


def run_cpp_and_report(
    exe: str | Path,
    *,
    cwd: str | Path = ".",
    output_dir: str | Path | None = None,
    title: str = "C++ 仿真结果报告",
    extra_args: list[str] | None = None,
) -> tuple[Path, Path]:
    cwd_path = Path(cwd)
    cmd = [str(exe)]
    if extra_args:
        cmd.extend(extra_args)
    completed = subprocess.run(
        cmd,
        cwd=cwd_path,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "C++ simulation failed with exit code "
            f"{completed.returncode}\nSTDOUT:\n{completed.stdout}\nSTDERR:\n{completed.stderr}"
        )
    sim_result = find_sim_result_path(completed.stdout + "\n" + completed.stderr, cwd_path)
    report_path = generate_cpp_result_report(sim_result, output_dir=output_dir, title=title)
    return sim_result, report_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="运行 C++ 仿真并自动生成中文图片报告。")
    parser.add_argument("--exe", required=True, help="C++ 仿真可执行文件路径。")
    parser.add_argument("--cwd", default=".", help="执行 C++ 程序的工作目录，默认当前目录。")
    parser.add_argument("--output-dir", default=None, help="报告输出目录；默认写到 C++ sim_result.json 所在目录。")
    parser.add_argument("--title", default="C++ 仿真结果报告", help="Markdown 报告标题。")
    parser.add_argument("args", nargs=argparse.REMAINDER, help="传给 C++ 可执行文件的额外参数。")
    args = parser.parse_args(argv)

    extra_args = args.args
    if extra_args and extra_args[0] == "--":
        extra_args = extra_args[1:]
    sim_result, report_path = run_cpp_and_report(
        args.exe,
        cwd=args.cwd,
        output_dir=args.output_dir,
        title=args.title,
        extra_args=extra_args,
    )
    print(f"C++ 结果: {sim_result}")
    print(f"C++ 报告: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["find_sim_result_path", "run_cpp_and_report"]
