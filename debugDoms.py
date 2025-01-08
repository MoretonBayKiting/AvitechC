import re
import os

def extract_lines(filename, patterns):
    results = []
    with open(filename, 'r') as file:
        for line in file:
            # Extract the part of the line starting from the time
            match = re.search(r'\d{1,2}:\d{2}:\d{2} \d{1,3}ms.*', line)
            if match:
                line_content = match.group(0)
                # Debug statement to print the extracted line content
                print(f"Extracted line content: {line_content}")
                # Check if the line matches any of the specified patterns
                for pattern in patterns:
                    if re.search(pattern, line_content):
                        results.append(line_content)
                        # Debug statement to print the matched line
                        print(f"Matched line: {line_content}")
                        break
    return results

def save_lines_to_file(output_filename, lines):
    with open(output_filename, 'w') as file:
        for line in lines:
            file.write(line + '\n')

if __name__ == '__main__':
    filename = 'debug/DomsLog1228G.txt'  # Replace with your input file name
    output_filename = os.path.splitext(filename)[0] + '_trim.txt'
    patterns = [
        r'<2:',
        r'<3:',
        r'<31:',
        r'<61:',
        r'<62:',
        r'<128:', r'<129:', r'<130:', r'<131:', r'<132:', r'<133:', r'<134:', r'<135:', r'<136:', r'<137:', r'<138:', r'<139:',
        r'<140:', r'<141:', r'<142:', r'<143:', r'<144:', r'<145:', r'<146:', r'<147:', r'<148:', r'<149:', r'<150:', r'<151:',
        r'<152:', r'<153:', r'<154:', r'<155:', r'<156:', r'<157:', r'<158:', r'<159:', r'<160:', r'<161:', r'<162:', r'<163:',
        r'<164:', r'<165:', r'<166:', r'<167:', r'<168:', r'<169:', r'<170:', r'<171:', r'<172:', r'<173:', r'<174:', r'<175:',
        r'<176:', r'<177:', r'<178:', r'<179:', r'<180:', r'<181:', r'<182:', r'<183:', r'<184:', r'<185:', r'<186:', r'<187:',
        r'<188:', r'<189:', r'<190:', r'<191:', r'<192:', r'<193:', r'<194:', r'<195:', r'<196:', r'<197:', r'<198:', r'<199:',
        r'<200:', r'<201:', r'<202:', r'<203:', r'<204:', r'<205:', r'<206:', r'<207:', r'<208:', r'<209:', r'<210:', r'<211:',
        r'<212:', r'<213:', r'<214:', r'<215:', r'<216:', r'<217:', r'<218:', r'<219:', r'<220:', r'<221:', r'<222:', r'<223:',
        r'<224:', r'<225:', r'<226:', r'<227:', r'<228:', r'<229:', r'<230:', r'<231:', r'<232:', r'<233:', r'<234:', r'<235:',
        r'<236:', r'<237:', r'<238:', r'<239:', r'<240:', r'<241:', r'<242:', r'<243:', r'<244:', r'<245:', r'<246:', r'<247:',
        r'<248:', r'<249:', r'<250:', r'<251:', r'<252:', r'<253:', r'<254:', r'<255:'
    ]
    extracted_lines = extract_lines(filename, patterns)
    save_lines_to_file(output_filename, extracted_lines)
    print(f"Extracted lines saved to {output_filename}")