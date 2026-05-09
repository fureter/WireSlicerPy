def get_r_and_c_from_num(num):

    ret_val = None
    for i in reversed(range(1, 20)):
        if num % i == 0:
            ret_val = (i, num / i)
            if ret_val is not None:
                aspect_ratio = ret_val[0] / ret_val[1]
                prev_aspect_ratio = (i+1) / (num % (i+1))

                if abs(1 - prev_aspect_ratio) > abs(1 - aspect_ratio):
                    break

    return ret_val


def cmd_to_byte_array(cmd):
    packet = bytearray()
    string_list = [cmd[i:i + 2] for i in range(0, len(cmd), 2)]
    for command in string_list:
        packet.append(int(command,16))
    return packet

def is_float(num):
    try:
        float(num)
        return True
    except ValueError:
        return False
