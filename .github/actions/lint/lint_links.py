## IMPORTS
import os
import sys
import re
import json
import yaml

## CONSTANTS
REGEX_PATTERN = r"(?<!!)\[.*?\]\(\s*https?:\/\/[^\(\)]+\)(?!\{\s*:?\s*target\s*=\s*(?:\s*_blank\s*|\s*\"\s*_blank\s*\"\s*)\})"
ROOT = "./docs/"
LINT_DIR = "./.github/workflows/lint.yml"
PASSED_MSG = "[PASSED]"
FAILED_MSG = "[FAILED]"
ERROR_MSG1 = "External links should redirect to a new tab. Change the link to "
ERROR_MSG2 = "{target=_blank}"

# Annotation strings for GitHub error annotations
annotations = []


## MAIN LOGIC
def main():

    # Perform the linting process
    ignore_files: list[str] = []
    ignore_patterns: list[str] = []
    config_dir: str = get_config_dir()
    if (config_dir and os.path.exists(config_dir)):
        ignore_files = get_ignore_files(config_dir)
        ignore_patterns = get_ignore_patterns(config_dir)
    markdown_files = get_markdown_files(ROOT, ignore_files)
    passed = lint_markdown_files(markdown_files, REGEX_PATTERN, ignore_patterns)

    # If linting fails, print any annotations to stderr for GitHub and exit with status code 1
    if not passed:
        print("\n".join(annotations), file=sys.stderr)
        sys.exit(1)


## HELPER FUNCTIONS
def get_config_dir() -> str:
    """
    Obtain the absolute path for the configuration file from lint.yml.

    Returns:
        str: The absolute path for the config file or an empty string if none is specified.
    """
    with open(LINT_DIR, 'r') as file:
        link_redirection_linter = yaml.safe_load(file)
    path: str = link_redirection_linter['jobs']['markdown-link-redirection-check']['steps'][3]['with']['config-file']
    file.close()
    return path


def get_ignore_files(dir: str) -> list[str]:
    """
    Obtain a list of files to ignore specified in the config file located in the specified directory

    Returns:
        List[str]: A list of markdown file paths to ignore when performing linting.
    """
    ignore_files = []
    f = open(dir)
    data = json.load(f)
    for row in data['ignoreFiles']:
        ignore_files.append(row['file'])
    f.close()
    return ignore_files


def get_ignore_patterns(dir: str) -> list[str]:
    """
    Obtain a list of patterns to ignore specified in the config file located in the specified directory.

    Returns:
        List[str]: A list of hyperlink patterns to ignore when performing linting.
    """
    ignore_patterns = []
    f = open(dir)
    data = json.load(f)
    for row in data['ignorePatterns']:
        ignore_patterns.append(row['pattern'])
    f.close()
    return ignore_patterns


def get_markdown_files(root_dir, ignore_files):
    """
    Recursively searches for markdown files (.md) starting at a specified root directory.

    Args:
        root_dir (str): The root directory to start the search at.
        ignore_files list[str]: A list of markdown file paths to ignore.

    Returns:
        List[str]: A list of markdown file paths relative to the root directory.
    """
    markdown_files = []
    markdown_matcher = re.compile(r".+\.md")
    for root, dirs, files in os.walk(root_dir):
        markdown_file_basenames = filter(lambda f: markdown_matcher.match(f) is not None, files)
        markdown_files_with_full_path = map(lambda f: os.path.join(root, f), markdown_file_basenames)
        markdown_files_to_keep = filter(lambda f: f not in ignore_files, markdown_files_with_full_path)
        markdown_files += list(markdown_files_to_keep)
    return markdown_files


def lint_markdown_files(files, pattern, ignore_patterns: list[str]):
    """
    Lints all specified markdown files and checks for any links to outside the Sailbot Docs website 
    that do not redirect to a new tab. If any such links exists, the linting process fails.

    Args:
        files (List[str]): A list of markdown file paths relative to some root directory.
        pattern (str): A raw string containing the regular expression pattern to be used for linting.
        ignore_patterns list[str]: A list of regex patterns to ignore.

    Returns:
        bool: Returns True if the linting process succeeds for all markdown files and False otherwise.
    """
    passed = True
    num_passed = 0
    num_checks = len(files)

    for n, file in enumerate(files):
        check_passed, error_message = check_markdown_file(file, pattern, ignore_patterns)
        passed = (passed and check_passed)
        num_passed += int(check_passed)
        print_check_message(file, check_passed, n+1, error_message)

    print(f"{num_passed}/{num_checks} checks passed")

    return passed


def check_markdown_file(filename, pattern, ignore_patterns: list[str]):
    """
    Lints a specified markdown file.

    Args:
        filename (str): The path to the markdown file relative to some root directory.
        pattern (str): A raw string containing the regular expression pattern to be used for linting.
        ignore_patterns list[str]: A list of regex patterns to ignore.

    Returns:
        tuple[bool, str]: Returns a tuple containing two variables:
            1. A boolean variable that indicates if the check passes
            2. A string containing an error message. This string is empty if the check passes.
    """
    passed = True
    error_message_buffer = ""

    with open(filename) as file:
        for line_number, line_text in enumerate(file.readlines()):
            match = re.findall(pattern, line_text, flags=re.M)
            ignore_links: list[str] = []
            for ignore_pattern in ignore_patterns:
                expression = re.compile(ignore_pattern)
                ignore_links += list(filter(lambda l: not expression.match(l), match))
            match = list(filter(lambda l: l not in ignore_links, match))
            if match:
                passed = False
                for link in match:
                    error_message_buffer += f"\tLine {line_number+1}: {link}\n"
                    annotations.append(f"::error file={filename},line={line_number+1}::{ERROR_MSG1 + link + ERROR_MSG2}")
    return passed, error_message_buffer


def print_check_message(filename, check_passed, check_number, error_message):
    """
    Prints the status of a markdown file after a check. Any errors will be printed along with
    the status.

    Args:
        filename (str): The path to the markdown file relative to some root directory.
        check_passed (bool): Whether the check on the markdown file passed or not.
        check_number (int): The check number.
        error_message (str): An error message (empty if the check passed).
    """
    status = PASSED_MSG if check_passed else FAILED_MSG
    print(f"Check {check_number}: {status} {filename}\n" + error_message)


if __name__ == '__main__':
    main()
