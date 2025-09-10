#!/usr/bin/env python3
"""
Script to reverse the order of lines in a text file.
"""

def reverse_file_lines(input_file, output_file=None):
    """
    Reverse the order of lines in a text file.
    
    Args:
        input_file (str): Path to the input file
        output_file (str, optional): Path to the output file. If None, overwrites input file.
    """
    try:
        # Read all lines from the file
        with open(input_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        # Reverse the order of lines
        reversed_lines = lines[::-1]
        
        # Determine output file path
        if output_file is None:
            output_file = input_file
        
        # Write reversed lines to output file
        with open(output_file, 'w', encoding='utf-8') as f:
            f.writelines(reversed_lines)
        
        print(f"Successfully reversed lines in '{input_file}'")
        if output_file != input_file:
            print(f"Output saved to '{output_file}'")
        
    except FileNotFoundError:
        print(f"Error: File '{input_file}' not found.")
    except PermissionError:
        print(f"Error: Permission denied when accessing '{input_file}'.")
    except Exception as e:
        print(f"Error: {e}")

def main():
    """Main function to handle command line usage."""
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python reverse_lines.py <input_file> [output_file]")
        print("  input_file:  Path to the text file to reverse")
        print("  output_file: Optional output file (if not provided, overwrites input)")
        return
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    reverse_file_lines(input_file, output_file)

if __name__ == "__main__":
    main()