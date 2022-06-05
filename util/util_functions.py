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


def is_float(num):
    try:
        float(num)
        return True
    except ValueError:
        return False
