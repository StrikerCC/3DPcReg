import copy
import json
import numpy as math
import numpy as np
import matplotlib.pyplot as plt

# statistics = {
#     'method': '',
#     'voxel_size': 0.0,
#     'time': 0.0,
#     'error_r': 0.0,
#     'error_t': 0.0,
#     'sub': [],
# }


# statistics_np = {
#     'method': '',
#     'voxel_size': 0.0,
#     'time': [],
#     'error_r': [],
#     'error_t': [],
#     'sub': [],
# }

class statistics_np:
    def __init__(self):
        self.method = ''
        self.voxel_size = 0.0
        self.time = []
        self.error_r = []
        self.error_t = []
        self.sub = []

    def mean(self):
        return {
            'time': np.mean(np.asarray(self.time)),
            'error_r': np.mean(np.asarray(self.error_r)),
            'error_t': np.mean(np.asarray(self.error_r)),
        }

    def std(self):
        return {
            'time': np.std(np.asarray(self.time)),
            'error_r': np.std(np.asarray(self.error_r)),
            'error_t': np.std(np.asarray(self.error_r)),
        }

    def plot(self):
        fig, (ax0, ax1, ax2) = plt.subplots(nrows=3, figsize=(9, 6))

        bins = 512
        ax0.hist(self.time, bins=bins, rwidth=0.9)
        ax1.hist(self.error_r, bins=bins, rwidth=0.9)
        ax2.hist(self.error_t, bins=bins, rwidth=0.9)

        ax0_x_low, ax0_x_up = np.min(self.time), np.max(self.time)
        ax1_x_low, ax1_x_up = np.min(self.error_r), np.max(self.error_r)
        ax2_x_low, ax2_x_up = np.min(self.error_t), np.max(self.error_t)

        ax0.set_xticks(np.arange(ax0_x_low, ax0_x_up, (ax0_x_up-ax0_x_low)/15))
        ax0.set_title('Time')
        ax0.set_xlabel('seconds')

        ax1.set_xticks(np.arange(ax1_x_low, ax1_x_up, (ax1_x_up-ax1_x_low)/15))
        ax1.set_title('Orientation error')
        ax1.set_xlabel('degree')

        ax2.set_xticks(np.arange(ax2_x_low, ax2_x_up, (ax2_x_up-ax2_x_low)/15))
        ax2.set_title('Distance error')
        ax2.set_xlabel('mm')

        fig.subplots_adjust(hspace=0.8)
        plt.title(self.method)
        plt.show()

    def __len__(self):
        assert len(self.time) == len(self.error_r) == len(self.error_t)
        return len(self.time)


def main():
    r_upper_bound_success = 2
    t_upper_bound_success = 0.8
    r_upper_bound_fail = 8
    t_upper_bound_fail = 10

    json_path = './00004.json'
    num_success: int = 0

    success = statistics_np()
    failure = statistics_np()

    success.method = 'Successful cases'
    failure.method = 'Failures'
    with open(json_path) as f:
        reg_results = json.load(f)
    reg_results = reg_results[:-2]

    '''average'''
    for reg_result in reg_results:
        error_r = float(reg_result['error_r'])
        error_t = float(reg_result['error_t'])
        time = float(reg_result['time'])
        if error_r < r_upper_bound_success and error_t < t_upper_bound_success:
            num_success += 1
            success.error_r.append(error_r)
            success.error_t.append(error_t)
            success.time.append(time)
        elif error_r < r_upper_bound_fail and error_t < t_upper_bound_fail:
            failure.error_r.append(error_r)
            failure.error_t.append(error_t)
            failure.time.append(time)

    print("success case ", num_success, "among ", len(reg_results), " case")
    print('success rate ', num_success / len(reg_results))
    print('success mean')
    print(success.mean())
    print('success std dev')
    print(success.std())
    success.plot()

    print('failure mean')
    print(failure.mean())
    print('failure std dev')
    print(failure.std())
    failure.plot()

    # for ele_mean in data.keys():
    #     print('     ', ele_mean, data[ele_mean])
    #     if ele_mean == 'sub':
    #         for sub_mean in data[ele_mean]:
    #             for ele_sub_mean in sub_mean.keys():
    #                 print('         ', ele_sub_mean, sub_mean[ele_sub_mean])
    #             print()
    #
    # print('stddev')
    # for ele_stddev in stddev.keys():
    #     print('     ', ele_stddev, stddev[ele_stddev])
    #     if ele_stddev == 'sub':
    #         for sub_stddev in stddev[ele_stddev]:
    #             for ele_sub_stddev in sub_stddev.keys():
    #                 print('         ', ele_sub_stddev, sub_stddev[ele_sub_stddev])
    #             print()
    #
    # print('failure')
    # for failure_ in failure:
    #     for ele_fail in failure_.keys():
    #         if ele_fail == 'statistics':
    #             for sub_fail in failure_[ele_fail]:
    #                 for ele_sub_fail in sub_fail.keys():
    #                     print('         ', ele_sub_fail, sub_fail[ele_sub_fail])
    #                 print()
    #         else:
    #             print('     ', ele_fail, failure_[ele_fail])
    #     print()


# def main():
#     r_upper_bound = 2
#     t_upper_bound = 0.2
#
#     json_path = './00015.json'
#     num_success: int = 0
#
#     mean = copy.deepcopy(statistics)
#     stddev = copy.deepcopy(statistics)
#     failure = []
#
#     with open(json_path) as f:
#         reg_results = json.load(f)
#     reg_results = reg_results[:-2]
#
#     '''average'''
#     for reg_result in reg_results:
#         error_r = float(reg_result['error_r_final'])
#         error_t = float(reg_result['error_t_final'])
#         time = float(reg_result['time_total'])
#         if error_r < r_upper_bound and error_t < t_upper_bound:
#             num_success += 1
#             mean['error_r'] += error_r
#             mean['error_t'] += error_t
#             mean['time'] += time
#             # mean['voxel_size'] = reg_result['voxel_size']
#             for i, reg_sub in enumerate(reg_result['statistics']):
#                 if len(mean['sub']) < len(reg_result['statistics']):
#                     mean['sub'].append(copy.deepcopy(statistics))
#                 sub = mean['sub'][i]
#                 sub['method'] = reg_sub['method']
#                 sub['voxel_size'] = reg_sub['voxel_size']
#                 sub['error_r'] += reg_sub['error_r']
#                 sub['error_t'] += reg_sub['error_t']
#                 sub['time'] += reg_sub['time']
#         else:
#             failure.append(reg_result)
#
#     mean['error_r'] /= num_success
#     mean['error_t'] /= num_success
#     mean['time'] /= num_success
#     for sub in mean['sub']:
#         sub['error_r'] /= num_success
#         sub['error_t'] /= num_success
#         sub['time'] /= num_success
#
#     '''stddev'''
#     for reg_result in reg_results:
#         error_r = float(reg_result['error_r_final'])
#         error_t = float(reg_result['error_t_final'])
#         time = float(reg_result['time_total'])
#         if error_r < r_upper_bound and error_t < t_upper_bound:
#             stddev['error_r'] += pow(error_r - mean['error_r'], 2)
#             stddev['error_t'] += pow(error_t - mean['error_t'], 2)
#             stddev['time'] += pow(time - mean['time'], 2)
#             for i, reg_sub in enumerate(reg_result['statistics']):
#                 if len(stddev['sub']) < len(reg_result['statistics']):
#                     stddev['sub'].append(copy.deepcopy(statistics))
#                 sub = stddev['sub'][i]
#                 sub['method'] = reg_sub['method']
#                 sub['voxel_size'] = reg_sub['voxel_size']
#                 sub['error_r'] += pow(reg_sub['error_r'] - mean['sub'][i]['error_r'], 2)
#                 sub['error_t'] += pow(reg_sub['error_t'] - mean['sub'][i]['error_t'], 2)
#                 sub['time'] += pow(reg_sub['time'] - mean['sub'][i]['time'], 2)
#
#
#     stddev['error_r'] = math.sqrt(stddev['error_r'] / num_success)
#     stddev['error_t'] = math.sqrt(stddev['error_t'] / num_success)
#     stddev['time'] = math.sqrt(stddev['time'] / num_success)
#     for sub in stddev['sub']:
#         sub['time'] = math.sqrt(sub['time'] / num_success)
#         sub['error_r'] = math.sqrt(sub['error_r'] / num_success)
#         sub['error_t'] = math.sqrt(sub['error_t'] / num_success)
#
#     print("success case ", num_success, "among ", len(reg_results), " case")
#     print('success rate ', num_success / len(reg_results))
#     print('mean')
#     for ele_mean in mean.keys():
#         print('     ', ele_mean, mean[ele_mean])
#         if ele_mean == 'sub':
#             for sub_mean in mean[ele_mean]:
#                 for ele_sub_mean in sub_mean.keys():
#                     print('         ', ele_sub_mean, sub_mean[ele_sub_mean])
#                 print()
#
#     print('stddev')
#     for ele_stddev in stddev.keys():
#         print('     ', ele_stddev, stddev[ele_stddev])
#         if ele_stddev == 'sub':
#             for sub_stddev in stddev[ele_stddev]:
#                 for ele_sub_stddev in sub_stddev.keys():
#                     print('         ', ele_sub_stddev, sub_stddev[ele_sub_stddev])
#                 print()
#
#     print('failure')
#     for failure_ in failure:
#         for ele_fail in failure_.keys():
#             if ele_fail == 'statistics':
#                 for sub_fail in failure_[ele_fail]:
#                     for ele_sub_fail in sub_fail.keys():
#                         print('         ', ele_sub_fail, sub_fail[ele_sub_fail])
#                     print()
#             else:
#                 print('     ', ele_fail, failure_[ele_fail])
#         print()


if __name__ == '__main__':
    main()
