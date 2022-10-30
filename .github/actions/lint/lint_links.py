import os
import sys
import re

## CONSTANTS
PASSED_MSG = '[PASSED]'
FAILED_MSG = '[FAILED]'
ERROR_MSG1 = 'External links should redirect to a new tab. Change the link to '
ERROR_MSG2 = "{target=_blank}"

## HELPER FUNCTIONS
def get_markdown_files(root_dir):
    """
    Recursively searches for markdown files (.md) starting at a specified root directory.

    Args:
        root_dir (str): The root directory to start the search at.

    Returns:
        List[str]: A list of markdown file paths relative to the root directory.
    """
    markdown_files = []
    markdown_matcher = re.compile(r".+\.md")
    for root, dirs, files in os.walk(root_dir):
        markdown_file_basenames = filter(lambda f: markdown_matcher.match(f) is not None, files)
        markdown_files_with_full_path = map(lambda f: os.path.join(root, f), markdown_file_basenames)
        markdown_files += list(markdown_files_with_full_path)
    return markdown_files


def lint_markdown_files(files, matcher):
    """
    Lints all specified markdown files and checks for any links to outside the Sailbot Docs website 
    that do not redirect to a new tab. If any such links exists, the linting process fails.

    Args:
        files (List[str]): A list of markdown file paths relative to some root directory.
        matcher (re.Pattern): A compiled regular expression object used to perform the linting.

    Returns:
        bool: Returns True if the linting process succeeds for all markdown files and False otherwise.
    """
    passed = True
    num_passed = 0
    num_checks = len(files)

    for n, file in enumerate(files):
        check_passed, error_message = check_markdown_file(file, matcher)
        passed = (passed and check_passed)
        num_passed += int(check_passed)
        print_check_message(file, check_passed, n+1, error_message)

    print(f"{num_passed}/{num_checks} checks passed")

    return passed


def check_markdown_file(filename, matcher):
    """
    Lints a specified markdown file.

    Args:
        filename (str): The path to the markdown file relative to some root directory.
        matcher (re.Pattern): A compiled regular expression object used to perform the linting.

    Returns:
        tuple[bool, str]: Returns a tuple containing two variables:
            1. A boolean variable that indicates if the check passes
            2. A string containing an error message. This string is empty if the check passes.
    """
    passed = True
    error_message_buffer = ""

    with open(filename) as file:
        for line_number, line_text in enumerate(file.readlines()):
            match = matcher.match(line_text)

            if match is not None:
                passed = False
                error_message_buffer += f"\tLine {line_number+1}: {match[0]}\n"

                print(f"::error file={filename},line={line_number+1}::{ERROR_MSG1 + match[0] + ERROR_MSG2}", file=sys.stderr)
    
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


## MAIN LOGIC
def main():
    root = './docs/'
    markdown_files = get_markdown_files(root)
    bad_link_matcher = re.compile(r"(?<!!)\[.*\]\(\s*https?:\/\/[^\(\)]+\)(?!\{\s*:?\s*target\s*=\s*(?:\s*_blank\s*|\s*\"\s*_blank\s*\"\s*)\})")
    passed = lint_markdown_files(markdown_files, bad_link_matcher)
    return passed

if __name__ == '__main__':
    passed = main()
    if not passed:
        sys.exit(1)
