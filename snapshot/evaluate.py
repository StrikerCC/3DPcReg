import copy
import os
import json
import math
import numpy as np
import matplotlib.pyplot as plt


class Statistics:
    def __init__(self):
        # self.method = ''
        # self.voxel_size = 0.0
        # self.noise_src = 0.0
        # self.num_points_src = []
        # self.num_points_tgt = []
        # self.time = []
        # self.error_r = []
        # self.error_t = []
        # self.children = []

        # self.data = {
        #     'method': self.method,
        #     'voxel_size': self.voxel_size,
        #     'noise_src': self.noise_src,
        #     'num_points_src': self.num_points_src,
        #     'num_points_tgt': self.num_points_tgt,
        #     'time': self.time,
        #     'error_r': self.error_r,
        #     'error_t': self.error_t,
        #     'children': [child.to_dict() for child in self.children],
        # }
        self.data = {
            'method': '',
            'voxel_size': 0.0,
            'noise_src': 0.0,
            'num_points_src': [],
            'num_points_tgt': [],
            'time': [],
            'error_r': [],
            'error_t': [],
            'children': [],
            'ratio': 0.0
        }

    def mean(self):
        mean = copy.deepcopy(self)
        mean.data['num_points_src'] = np.mean(np.asarray(self.data['num_points_src']))
        mean.data['num_points_tgt'] = np.mean(np.asarray(self.data['num_points_tgt']))
        mean.data['time'] = np.mean(np.asarray(self.data['time']))
        mean.data['error_r'] = np.mean(np.asarray(self.data['error_r']))
        mean.data['error_t'] = np.mean(np.asarray(self.data['error_t']))
        mean.data['children'] = [sub_.mean() for sub_ in mean.data['children']]
        return mean

    def stddev(self):
        stddev = copy.deepcopy(self)
        stddev.data['num_points_src'] = np.std(np.asarray(self.data['num_points_src']))
        stddev.data['num_points_tgt'] = np.std(np.asarray(self.data['num_points_tgt']))
        stddev.data['time'] = np.std(np.asarray(self.data['time']))
        stddev.data['error_r'] = np.std(np.asarray(self.data['error_r']))
        stddev.data['error_t'] = np.std(np.asarray(self.data['error_t']))
        stddev.data['children'] = [sub_.stddev() for sub_ in stddev.data['children']]
        return stddev

    def plot(self):
        figure_name = self.data['method'] + ' successful rate: ' + str(round(self.data['ratio'], 4)) + '%, voxel size: ' + str(self.data['voxel_size']) + 'mm, noise sigma: ' + \
                      str(self.data['noise_src']) + 'mm'

        fig, (ax0, ax1, ax2) = plt.subplots(nrows=3, figsize=(9, 6))
        fig.suptitle(figure_name)
        # build histogram
        bins = 512
        ax0.hist(self.data['time'], bins=bins, rwidth=0.9)
        ax1.hist(self.data['error_r'], bins=bins, rwidth=0.9)
        ax2.hist(self.data['error_t'], bins=bins, rwidth=0.9)

        # setup figure detail
        # ax0_x_low, ax0_x_up = np.min(self.data['time']), np.max(self.data['time'])
        # ax1_x_low, ax1_x_up = np.min(self.data['error_r']), np.max(self.data['error_r'])
        # ax2_x_low, ax2_x_up = np.min(self.data['error_t']), np.max(self.data['error_t'])

        ax0_x_low, ax0_x_up = 0, np.max(self.data['time'])
        ax1_x_low, ax1_x_up = 0, np.max(self.data['error_r'])
        ax2_x_low, ax2_x_up = 0, np.max(self.data['error_t'])

        # ax0.set_xticks(np.arange(ax0_x_low, ax0_x_up, (ax0_x_up - ax0_x_low) / 20))
        ax0.set_xticks(np.arange(ax0_x_low, ax0_x_up, 1))
        ax0.set_title('Time')
        ax0.set_xlabel('seconds')

        # ax1.set_xticks(np.arange(ax1_x_low, ax1_x_up, (ax1_x_up - ax1_x_low) / 20))
        ax1.set_xticks(np.arange(ax1_x_low, ax1_x_up, 0.01))
        ax1.set_title('Orientation error')
        ax1.set_xlabel('degree')

        # ax2.set_xticks(np.arange(ax2_x_low, ax2_x_up, (ax2_x_up - ax2_x_low) / 20))
        ax2.set_xticks(np.arange(ax2_x_low, ax2_x_up, 0.02))
        ax2.set_title('Distance error')
        ax2.set_xlabel('mm')

        fig.subplots_adjust(hspace=0.8)
        # plt.title()

        plt.show()

    def to_dict(self):
        return copy.deepcopy(self.data)

    def __len__(self):
        assert len(self.data['time']) == len(self.data['error_r']) == len(self.data['error_t'])
        return len(self.data['time'])

    def load_json_file(self, json_path='./00004.json'):
        with open(json_path) as f:
            reg_results = json.load(f)
        return self.load_dict(reg_results)

    def load_dict(self, reg_results):
        for reg_result in reg_results:
            # get key parameters
            method = reg_result['method']
            voxel_size = reg_result['voxel_size']
            noise_src = reg_result['noise_src']
            num_points_src = reg_result['num_points_src']
            num_points_tgt = reg_result['num_points_tgt']
            time = float(reg_result['time'])
            error_r = float(reg_result['error_r'])
            error_t = float(reg_result['error_t'])
            reg_result_children = reg_result['statistics_step']
            # format key parameters
            digits = 6
            voxel_size = round(voxel_size, digits)
            noise_src = round(noise_src, digits)
            num_points_src = int(num_points_src)
            num_points_tgt = int(num_points_tgt)

            self.data['method'] = method
            self.data['voxel_size'] = voxel_size
            self.data['noise_src'] = noise_src
            self.data['num_points_tgt'].append(num_points_tgt)
            self.data['num_points_src'].append(num_points_src)
            self.data['error_r'].append(error_r)
            self.data['error_t'].append(error_t)
            self.data['time'].append(time)

    # def __str__(self):
    #     prefix = '  '
    #     cout = ''
    #     cout += prefix + 'method: ' + self.method + '\n'
    #     cout += prefix + 'voxel_size: ' + str(self.voxel_size) + '\n'
    #     cout += prefix + 'noise: ' + str(self.noise_src) + '\n'
    #     cout += prefix + 'num_points_src: ' + str(self.num_points_src) + '\n'
    #     cout += prefix + 'num_points_tgt: ' + str(self.num_points_tgt) + '\n'
    #     cout += prefix + 'time: ' + str(self.time) + '\n'
    #     cout += prefix + 'error_r: ' + str(self.error_r) + '\n'
    #     cout += prefix + 'error_t: ' + str(self.error_t) + '\n'
    #     for sub_ in self.children:
    #         cout += prefix + '' + str(sub_) + '\n'
    #     return cout


class StatisticsMultiDim:
    def __init__(self):
        self.statistics_ = {}
        '''
        "statistics": {
            "voxel_size: 0.1": {
                "noise_src: 0.02": {
                    "ranks" : {reg_result}
                }
            }
        }
    
        '''
        self.r_upper_bounds = (2, 8)  # rotation error to rank reg result (smaller to bigger)
        self.t_upper_bounds = (0.8, 10)  # translation error to rank reg result (smaller to bigger)
        self.r_upper_bounds = (0.5,)  # rotation error to rank reg result (smaller to bigger)
        self.t_upper_bounds = (0.8,)  # translation error to rank reg result (smaller to bigger)
        self.json_path = ''
        # self.r_upper_bound_fail = 8
        # self.t_upper_bound_fail = 10

    def __len__(self):
        return 0

    def load_json(self, json_path='./00004.json'):
        self.json_path = json_path

        with open(json_path) as f:
            reg_results = json.load(f)
        reg_results = reg_results[:-2]

        '''average'''
        for reg_result in reg_results:
            # get key parameters
            # method = reg_result['method']
            voxel_size = reg_result['voxel_size']
            noise_src = reg_result['noise_src']
            # num_points_src = int(reg_result['num_points_src'])
            # num_points_tgt = int(reg_result['num_points_tgt'])
            # time = float(reg_result['time'])
            error_r = float(reg_result['error_r'])
            error_t = float(reg_result['error_t'])
            reg_result_children = reg_result['statistics_step']
            key_rank = -1

            # format key parameters
            digits = 5
            voxel_size = round(voxel_size, digits)
            noise_src = round(noise_src, digits)

            for i_error_interval, (r_upper_bound, t_upper_bound) in enumerate(
                    zip(self.r_upper_bounds, self.t_upper_bounds)):  # rank key the result according to error
                if error_r < r_upper_bound and error_t < t_upper_bound:
                    key_rank = i_error_interval
                    break
            if key_rank == -1:
                key_rank = len(self.r_upper_bounds)

            # build dict for different setting
            if voxel_size not in self.statistics_.keys():
                self.statistics_[voxel_size] = {}
            if noise_src not in self.statistics_[voxel_size].keys():
                self.statistics_[voxel_size][noise_src] = {}
            if key_rank not in self.statistics_[voxel_size][noise_src].keys():
                self.statistics_[voxel_size][noise_src][key_rank] = []

            '''copy data'''
            self.statistics_[voxel_size][noise_src][key_rank].append(reg_result)

        '''load data into statistics'''
        for key_voxel_size in self.statistics_.keys():
            for key_noise_src in self.statistics_[key_voxel_size].keys():
                # compute ration of each rank
                num_cases = 0
                for key_rank, reg_result in self.statistics_[key_voxel_size][key_noise_src].items():
                    num_cases += len(reg_result)
                if num_cases == 0: break

                # build statistic
                for key_rank, result_rank in self.statistics_[key_voxel_size][key_noise_src].items():
                    statistics_rank = Statistics()
                    statistics_rank.load_dict(result_rank)
                    statistics_rank.data['ratio'] = len(result_rank) / num_cases
                    self.statistics_[key_voxel_size][key_noise_src][key_rank] = statistics_rank
        ''''''

    def mean(self):
        statistics_multi_dim_mean = copy.deepcopy(self)
        mean_statistics = statistics_multi_dim_mean.statistics_
        self.__mean_helper(mean_statistics)
        return statistics_multi_dim_mean

    def __mean_helper(self, dict_statistics):
        for key, value in dict_statistics.items():
            if not isinstance(value, Statistics):
                self.__mean_helper(value)
            else:  # base case
                dict_statistics[key] = dict_statistics[key].mean()

    def stddev(self):
        statistics_multi_dim_stddev = copy.deepcopy(self)
        mean_statistics = statistics_multi_dim_stddev.statistics_
        self.__stddev_helper(mean_statistics)
        return statistics_multi_dim_stddev

    def __stddev_helper(self, dict_statistics):
        for key, value in dict_statistics.items():
            if not isinstance(value, Statistics):
                self.__stddev_helper(value)
            else:  # base case
                dict_statistics[key] = dict_statistics[key].stddev()

    def to_dict(self):
        self_dict = {'r_upper_bounds': self.r_upper_bounds,
                     't_upper_bounds': self.t_upper_bounds,
                     'statistics': copy.deepcopy(self.statistics_)}
        self.__to_dict_helper(self_dict)
        return self_dict

    def __to_dict_helper(self, statistics_dict):
        for key, value in statistics_dict.items():
            if not isinstance(value, Statistics):
                if not isinstance(value, dict):
                    continue
                else:
                    self.__to_dict_helper(value)
            else:
                statistics_dict[key] = value.to_dict()

    def plot(self):
        fig_root = self.json_path[:-4] + 'vis/'
        if not os.path.exists(fig_root):
            os.mkdir(fig_root)

        mean = self.mean()
        std = self.stddev()

        width = len(mean.statistics_[list(mean.statistics_.keys())[0]].keys())
        height = len(mean.statistics_.keys())
        # xs = [[0 for j in range(width)] for i in range(height)]
        xs = np.zeros((height, width))
        ys, ratio_0, time_0, mean_r_0, mean_t_0, std_r_0, std_t_0 = copy.deepcopy(xs), copy.deepcopy(xs), copy.deepcopy(xs), copy.deepcopy(xs), copy.deepcopy(xs), copy.deepcopy(xs), copy.deepcopy(xs)

        for i, key_voxel_size in enumerate(mean.statistics_.keys()):
            for j, key_noise_src in enumerate(mean.statistics_[key_voxel_size].keys()):
                key_rank = 0
                result_rank = mean.statistics_[key_voxel_size][key_noise_src][key_rank]
                xs[i][j] = key_voxel_size
                ys[i][j] = key_noise_src
                ratio_0[i][j] = result_rank.data['ratio']
                time_0[i][j] = result_rank.data['time']
                mean_r_0[i][j] = result_rank.data['error_r']
                mean_t_0[i][j] = result_rank.data['error_t']

        fig = plt.figure()
        ax3 = plt.axes(projection='3d')
        # ax3.scatter3D(xs, ys, ratio_0, cmap='rank 0 ratio')
        # ax3.plot_surface(xs, ys, ratio_0) #, cmap='rank 0 ratio')
        ax3.plot_wireframe(xs, ys, ratio_0) #, label='Successful rate')
        for x, y, z in zip(xs.flatten(), ys.flatten(), ratio_0.flatten()):
            anno = round(z, 5)
            ax3.text(x, y, z, anno, color='red')

        ax3.set_xlabel('voxel size')
        ax3.set_ylabel('noise sigma')
        ax3.set_zlabel('percent')
        plt.title('Successful rate')
        ax3.legend()
        plt.show()
        fig.savefig(fig_root + 'SuccessfulRateVsVoxelSizAndNoiseSigma.png')

        fig = plt.figure()
        ax3 = plt.axes(projection='3d')
        # ax3.scatter3D(xs, ys, ratio_0, cmap='rank 0 ratio')
        # ax3.plot_surface(xs, ys, ratio_0) #, cmap='rank 0 ratio')
        ax3.plot_wireframe(xs, ys, time_0) #, label='Successful rate')
        for x, y, z in zip(xs.flatten(), ys.flatten(), time_0.flatten()):
            anno = round(z, 5)
            ax3.text(x, y, z, anno, color='red')
        ax3.set_xlabel('voxel size')
        ax3.set_ylabel('noise sigma')
        ax3.set_zlabel('seconds')
        plt.title('Average time ')
        ax3.legend()
        plt.show()
        fig.savefig(fig_root + 'AverageTimeVsVoxelSizAndNoiseSigma.png')


        fig = plt.figure()
        ax3 = plt.axes(projection='3d')
        ax3.plot_wireframe(xs, ys, mean_r_0) #, label='mean rotation error')
        for x, y, z in zip(xs.flatten(), ys.flatten(), mean_r_0.flatten()):
            anno = round(z, 5)
            ax3.text(x, y, z, anno, color='red')
        ax3.legend()
        ax3.set_xlabel('voxel size')
        ax3.set_ylabel('noise sigma')
        ax3.set_zlabel('degree')
        plt.title('Average rotation error')
        plt.show()
        fig.savefig(fig_root + 'AverageRotationErrorVsVoxelSizAndNoiseSigma.png')


        fig = plt.figure()
        ax3 = plt.axes(projection='3d')
        ax3.plot_wireframe(xs, ys, mean_t_0) #, label='mean translation error')
        for x, y, z in zip(xs.flatten(), ys.flatten(), mean_t_0.flatten()):
            anno = round(z, 5)
            ax3.text(x, y, z, anno, color='red')
        ax3.legend()
        ax3.set_xlabel('voxel size')
        ax3.set_ylabel('noise sigma')
        ax3.set_zlabel('mm')
        plt.title('Average translation error')
        plt.show()
        fig.savefig(fig_root + 'AverageTranslationErrorVsVoxelSizAndNoiseSigma.png')

        for i, key_voxel_size in enumerate(std.statistics_.keys()):
            for j, key_noise_src in enumerate(std.statistics_[key_voxel_size].keys()):
                key_rank = 0
                result_rank = std.statistics_[key_voxel_size][key_noise_src][key_rank]

                xs[i][j] = key_voxel_size
                ys[i][j] = key_noise_src
                ratio_0[i][j] = result_rank.data['ratio']
                std_r_0[i][j] = result_rank.data['error_r']
                std_t_0[i][j] = result_rank.data['error_t']

        fig = plt.figure()
        ax3 = plt.axes(projection='3d')
        ax3.plot_wireframe(xs, ys, std_r_0) #, label='rotation error stddev')
        for x, y, z in zip(xs.flatten(), ys.flatten(), std_r_0.flatten()):
            anno = round(z, 5)
            ax3.text(x, y, z, anno, color='red')
        ax3.legend()
        ax3.set_xlabel('voxel size')
        ax3.set_ylabel('noise sigma')
        ax3.set_zlabel('degree')
        plt.title('Rotation error standard deviation')
        plt.show()

        fig = plt.figure()
        ax3 = plt.axes(projection='3d')
        ax3.plot_wireframe(xs, ys, std_t_0) #, label='translation error stddev')
        for x, y, z in zip(xs.flatten(), ys.flatten(), std_t_0.flatten()):
            anno = round(z, 5)
            ax3.text(x, y, z, anno, color='red')
        ax3.legend()
        ax3.set_xlabel('voxel size')
        ax3.set_ylabel('noise sigma')
        ax3.set_zlabel('mm')
        plt.title('Translation error standard deviation')
        plt.show()

    def plot_hist(self):
        for i, key_voxel_size in enumerate(self.statistics_.keys()):
            for j, key_noise_src in enumerate(self.statistics_[key_voxel_size].keys()):
                key_rank = 0
                result_rank = self.statistics_[key_voxel_size][key_noise_src][key_rank]

                '''plot each 0 rank result statistics'''
                result_rank.plot()

    # def __str__(self):
    #     return self.__str_helper(self.statistics_)

    # def __str_helper(self, statistics_dict, cout='', prefix=''):
    #     prefix_next = '     '
    #     for key, value in statistics_dict.items():
    #         if not isinstance(value, Statistics):
    #             cout += prefix + str(key) + '\n'
    #             cout += self.__str_helper(value, cout, prefix + prefix_next)
    #         else:
    #             cout += prefix + str(key) + '\n'
    #             cout += prefix + prefix_next + str(value) + '\n'
    #     return cout


def main():
    statistics_ = Statistics()
    statistics_.load_json_file()
    mean_homo = statistics_.mean()
    stddev_homo = statistics_.stddev()
    mean_dict_homo = mean_homo.to_dict()
    stddev_dict_homo = stddev_homo.to_dict()

    statistics_mMulti_dim = StatisticsMultiDim()
    statistics_mMulti_dim.load_json()
    # statistics_mMulti_dim.plot_hist()

    mean = statistics_mMulti_dim.mean()
    stddev = statistics_mMulti_dim.stddev()

    statistics_mMulti_dim.plot()

    mean_dict = mean.to_dict()
    stddev_dict = stddev.to_dict()

    with open('00004_analysis.json', 'w') as f:
        json.dump({'mean_homo': mean_dict_homo,
                   'stddev_homo': stddev_dict_homo,
                   'mean': mean_dict,
                   'stddev': stddev_dict}, f)


def main_0():
    r_upper_bound_success = 2
    t_upper_bound_success = 0.8
    r_upper_bound_fail = 8
    t_upper_bound_fail = 10

    json_path = './00004.json'
    num_success: int = 0

    success = Statistics()
    failure = Statistics()

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
    print(success.stddev())
    success.plot()

    print('failure mean')
    print(failure.mean())
    print('failure std dev')
    print(failure.stddev())
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
