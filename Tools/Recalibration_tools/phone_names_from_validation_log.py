import re
from collections import Counter


input_file = 'validation_results.csv'

start_mask = '\.com-'
end_mask = '##'

phone_list = []

with open(input_file, 'r') as val_log:
    lines = val_log.readlines()
    for l in lines:
        elements = l.split(',')
        name = elements[0]

        start = re.search(start_mask, name)
        end = re.search(end_mask, name)

        if start is not None and end is not None:
            i_start = start.regs[0][1]
            i_end = end.regs[0][0]

            # print(name[i_start:i_end])
            phone_list.append(name[i_start:i_end])

print(Counter(phone_list))

unique_phone_list = list(set(phone_list))

for phone in unique_phone_list:
    print(phone)

