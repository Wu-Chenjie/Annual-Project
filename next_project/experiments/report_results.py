from __future__ import annotations

import argparse

from experiments.result_reporter import generate_result_report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="从 sim_result.json 生成中文图片报告。")
    parser.add_argument("input_json", help="输入 sim_result.json 路径。")
    parser.add_argument("--output-dir", default=None, help="报告输出目录；默认写到输入文件所在目录。")
    parser.add_argument("--title", default=None, help="Markdown 报告标题。")
    args = parser.parse_args(argv)

    report_path = generate_result_report(args.input_json, args.output_dir, title=args.title)
    print(f"报告已生成: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
