## IMPORTS
import os
import sys
import re
import json

## CONSTANTS
REGEX_PATTERN = r"(?<!!)\[.*?\]\(\s*https?:\/\/[^\(\)]+\)(?!\{\s*:?\s*target\s*=\s*(?:\s*_blank\s*|\s*\"\s*_blank\s*\"\s*)\})"
LINK_REGEX = r"https?:\/\/(www\.)?[-a-zA-Z0-9@:%._\+~#=]{1,256}\.[a-zA-Z0-9()]{1,6}\b([-a-zA-Z0-9()@:%_\+.~#?&//=]*)"
ROOT = os.environ.get("ROOT", "./")
PASSED_MSG = "[PASSED]"
FAILED_MSG = "[FAILED]"
ERROR_MSG1 = "External links should redirect to a new tab. Change the link to "
ERROR_MSG2 = "{target=_blank}"

# Annotation strings for GitHub error annotations
annotations = []


## MAIN LOGIC
def main():

    # Perform the linting process
    ignore_patterns = []
    config_file = os.environ.get("CONFIG_FILE")
    ignore_patterns = get_ignore_patterns(config_file)
    ignore_files = get_ignore_files()
    markdown_files = get_markdown_files(ROOT, ignore_files)
    passed = lint_markdown_files(markdown_files, REGEX_PATTERN, ignore_patterns)

    # If linting fails, print any annotations to stderr for GitHub and exit with status code 1
    if not passed:
        print("\n".join(annotations), file=sys.stderr)
        sys.exit(1)


## HELPER FUNCTIONS
def get_ignore_patterns(config_file):
    """
    Obtain a list of patterns to ignore specified in the config file whose path is specified in lint.yml.

    Args:
        config_file (str): The path of the config file relative to the root.

    Returns:
        List[str]: A list of regex patterns to ignore when performing linting.
    """
    ignore_patterns = []
    if config_file and os.path.isfile(config_file):
        with open(config_file) as f:
            data = json.load(f)
            for row in data["ignorePatterns"]:
                ignore_patterns.append(row["pattern"])
    return ignore_patterns


def get_ignore_files():
    """
    Obtain a list of files to ignore specified in the environment variable.

    Returns:
        List[str]: A list of markdown file paths (relative to the root) to ignore when performing linting.
    """
    files = os.environ.get("IGNORE_FILES", "").split(" ")
    ignore_files_paths = []
    for file_name in files:
        file_path = os.path.join(ROOT, file_name)
        if os.path.isfile(file_path):
            ignore_files_paths.append(file_path)
    return ignore_files_paths


def get_markdown_files(root_dir, ignore_files):
    """
    Recursively searches for markdown files (.md) starting at a specified root directory.

    Args:
        root_dir (str): The root directory to start the search at.
        ignore_files (List[str]): A list of markdown file paths to ignore.

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


def lint_markdown_files(files, pattern, ignore_patterns):
    """
    Lints all specified markdown files and checks for any links to outside the Sailbot Docs website
    that do not redirect to a new tab. If any such links exists, the linting process fails.

    Args:
        files (List[str]): A list of markdown file paths relative to some root directory.
        pattern (str): A raw string containing the regular expression pattern to be used for linting.
        ignore_patterns (List[str]): A list of regex patterns to ignore.

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


def check_markdown_file(filename, pattern, ignore_patterns):
    """
    Lints a specified markdown file.

    Args:
        filename (str): The path to the markdown file relative to some root directory.
        pattern (str): A raw string containing the regular expression pattern to be used for linting.
        ignore_patterns (List[str]): A list of regex patterns to ignore.

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
            ignore_links = []
            for ignore_pattern in ignore_patterns:
                expression = re.compile(ignore_pattern)
                ignore_links += list(filter(lambda l: expression.match(re.search(LINK_REGEX, l).group()[:-1]), match))
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
