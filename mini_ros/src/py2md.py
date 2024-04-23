#py_to_md.py
import os

def check_and_generate_md():
    current_dir = os.getcwd()
    py_files = [file for file in os.listdir(current_dir) if file.endswith(".py") and file != "py_to_md.py"]
    code_blocks = []
    
    for py_file in py_files:
        with open(py_file, 'r') as file:
            lines = file.readlines()
            if lines and lines[0].strip() != f'#{py_file}':
                lines.insert(0, f'#{py_file}\n')
                print(f"Added missing line in {py_file}")
            
            code_blocks.append(f"```python\n{''.join(lines)}\n\n\n```")

    md_content = "\n\n\n\n\n\n".join(code_blocks)
    with open('code.md', 'w') as md_file:
        md_file.write(md_content)

    print("code.md file generated successfully!")


if __name__ == "__main__":
    # Change the working directory to the directory of this file
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    check_and_generate_md()