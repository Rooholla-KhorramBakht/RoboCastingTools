import numpy as np
from tqdm import tqdm

class imuModel():
    '''
    imuModel class converts the raw IMU data to scale calibrated 
    values. The parameters are estimated using the IMU-TK calibration 
    package. 

    ...

    Attributes
    -----------

    Methods
    ----------
    getCalibrated(accel_raw, gyro_raw)
        computes the calibrated accelerometer and gyroscope from a list of raw inputs.

    getCalibratedOnce(accel_raw, gyro_raw)
        compute the calibrated accelerometer and gyroscope from a single raw input.

    extractCalibParams(calibration_file)
        extract the calibration matrices from the IMU-TK result files

    '''
    
    def __init__(self, gyro_calib_file, accel_calib_file):
        '''
        Parameters
        -----------
        gyro_calib_file : str
            The path to the gyroscope calibration file 
            computed using the IMU-TK package found here: 
            https://github.com/Rooholla-KhorramBakht/imu_tk.git

        accel_calib_file : str
            The path to the accelerometer calibration file 
            computed using the IMU-TK package found here: 
            https://github.com/Rooholla-KhorramBakht/imu_tk.git

        '''
        self.Ta, self.Ka, self.ba = \
            self.extractCalibParams(accel_calib_file)

        self.Tg, self.Kg, self.bg = \
            self.extractCalibParams(gyro_calib_file)

    def getCalibrated(self, accel_raw, gyro_raw):
        '''
        Parameters
        -----------
        accel_raw : Numpy array
            Nx3 raw accelerometer samples

        gyro_raw : Numpy array
            Nx3 raw gyroscope samples

        Returns
        -----------
        (accel_calib, gyro_calib): a tuple of calibrated samples

        '''
        acc_calib  = (self.Ta@self.Ka@(accel_raw.T - self.ba)).T
        gyro_calib = (self.Tg@self.Kg@(gyro_raw.T - self.bg)).T
        return acc_calib, gyro_calib
    
    def getCalibratedOne(self, accel_raw, gyro_raw):
        '''
        Parameters
        -----------
        accel_raw : Numpy array
            3x1 single raw accelerometer sample

        gyro_raw : Numpy array
            3x1 single gyroscope sample

        Returns
        -----------
        (accel_calib, gyro_calib): a tuple of calibrated sample
        '''
        acc_calib  = (self.Ta@self.Ka@(accel_raw - self.ba)).T
        gyro_calib = (self.Tg@self.Kg@(gyro_raw - self.bg)).T
        return acc_calib.squeeze(), gyro_calib.squeeze()


    def extractCalibParams(self, calibration_file):
        '''
        Parameters
        -----------
        calibration_file : str
            path to the calibration file from IMU-TK

        Returns
        -----------
        (T, K, b): Tuple of scale calibration matrices
        '''
        with open(calibration_file, 'r') as f:
            data = f.readlines()
        data = [d.strip('\n') for d in data]
        float_data = []
        for data_lines in data:
            if len(data_lines)>0:
                float_data.append(np.fromstring(data_lines, dtype=float, sep=' '))
        T = np.vstack(float_data[0:3])    
        K = np.vstack(float_data[3:6])    
        b = np.vstack(float_data[6:9])  
        return T, K, b

def RosBagImuPost(input_bag, output_bag, imu_topic, imu_model):
    '''
    RosBagImuPost gets a rosbag of raw IMU samples and generates another bag with
    calibrated and scaled IMU data. 

    :param: input_bag: the path to the input bag file
    :param: output_bag: the path to the output bag file
    :param: imu_topic: the name of the imu topic in the input and output bag files
    :param: imu_model: the imuModel class that processes the raw imu data 
    '''
    for topic, msg, t in tqdm(bag.read_messages(topics=['/aras_usb_imu/vio'])):
        accel_raw = np.array(msg.acc).reshape(3,1)
        gyro_raw = np.array(msg.gyr).reshape(3,1)
        accel, gyro = imu.getCalibratedOne(accel_raw, gyro_raw)
        imu_msg.header = msg.header
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        bag_out.write('/imu', imu_msg, t=t)