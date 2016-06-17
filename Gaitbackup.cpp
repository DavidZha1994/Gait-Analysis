//================================
// Name        : hello world.cpp
// Author      :
// Version     : 2.3
//================================

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnCppWrapper.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include <time.h>
#include <math.h>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "/home/ilab/kinect/openni/Samples/Config/SamplesConfig.xml"
#define SAMPLE_XML_PATH_LOCAL "SamplesConfig.xml"
#define MAX_NUM_USERS 10
#define NUM 60
#define MAXLINE 100
#define LISTENQ 1024 /* 2nd argument to listen() */
#define SERV_PORT 9877
#define SA struct sockaddr

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::ScriptNode g_scriptNode;
xn::UserGenerator g_UserGenerator;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";

//---------------------------------------------------------------------------
// Function
//---------------------------------------------------------------------------
unsigned long GetTickCount() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

double PythagoreanTheorem(XnSkeletonJointTransformation a,
	XnSkeletonJointTransformation b) {
	return sqrt(
		pow(a.position.position.X - b.position.position.X, 2)
		+ pow(a.position.position.Y - b.position.position.Y, 2));
}

void InsertSort(double arr[], int n) {
	double temp;
	int i, j;
	for (i = 0; i < n; i++) {
		temp = arr[i];
		j = i - 1;
		while (j >= 0 && temp < arr[j]) {
			arr[j + 1] = arr[j];
			j = j - 1;
		}
		arr[j + 1] = temp;
	}
}

static int my_read(FILE *fp, char *ptr) {
	static int read_cnt = 0;
	static char *read_ptr;
	static char read_buf[MAXLINE];

	if (read_cnt <= 0) {
	again: if ((read_cnt = fread(read_buf, sizeof(char), 20, fp)) < 0) {
		if (errno == EINTR)
			goto again;
		return -1;
	}
		   else if (read_cnt == 0)
			   return 0;
		   read_ptr = read_buf;
	}
	read_cnt--;
	*ptr = *read_ptr++;
	return 1;
}

int readline(FILE *fp, char *vptr, int maxlen) {
	int n;
	int rc;
	char c;
	char *ptr = vptr;

	for (n = 1; n < maxlen; n++) {
		if ((rc = my_read(fp, &c)) == 1) {
			*ptr++ = c;
			if (c == '\n')
				break; /* newline is stored, like fgets() */
		}
		else if (rc == 0) {
			if (n == 1)
				return 0; /* EOF, no data read */
			else
				break; /* EOF, some data was read */
		}
		else
			return -1; /* error, errno set by read() */
	}
	*ptr = 0; /* null terminate like fgets() */
	return n;
}

XnBool fileExists(const char *fn) {
	XnBool exists;
	xnOSDoesFileExist(fn, &exists);
	return exists;
}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& /*generator*/,
	XnUserID nId, void* /*pCookie*/) {
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d New User %d\n", epochTime, nId);
	// New user found
	if (g_bNeedPose) {
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose,
			nId);
	}
	else {
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& /*generator*/,
	XnUserID nId, void* /*pCookie*/) {
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Lost user %d\n", epochTime, nId);
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(
	xn::PoseDetectionCapability& /*capability*/, const XnChar* strPose,
	XnUserID nId, void* /*pCookie*/) {
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Pose %s detected for user %d\n", epochTime, strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(
	xn::SkeletonCapability& /*capability*/, XnUserID nId,
	void* /*pCookie*/) {
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Calibration started for user %d\n", epochTime, nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(
	xn::SkeletonCapability& /*capability*/, XnUserID nId,
	XnCalibrationStatus eStatus, void* /*pCookie*/) {
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	if (eStatus == XN_CALIBRATION_STATUS_OK) {
		// Calibration succeeded
		printf("%d Calibration complete, start tracking user %d\n", epochTime,
			nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		// Calibration failed
		printf("%d Calibration failed for user %d\n", epochTime, nId);
		if (eStatus == XN_CALIBRATION_STATUS_MANUAL_ABORT) {
			printf("Manual abort occured, stop attempting to calibrate!");
			return;
		}
		if (g_bNeedPose) {
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose,
				nId);
		}
		else {
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
#define CHECK_RC(nRetVal, what)					    \
	if (nRetVal != XN_STATUS_OK)				    \
	{								    \
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));    \
		return nRetVal;						    \
	}

int main() {
	XnStatus nRetVal = XN_STATUS_OK;
	XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete,
		hPoseDetected;
	XnUserID aUsers[MAX_NUM_USERS];
	XnUInt16 nUsers;
	XnSkeletonJointTransformation joint_1, joint_2, joint_3, joint_4, joint_5,
		joint_6;
	xn::EnumerationErrors errors;
	const char *fn = NULL;

	int flag = 0;  // 终止记录标志
	double rec = 0;

	struct record {
		XnUInt16 userNo; // 当前正在检测的骨架编号
		int n;           // 已记录帧数
		int userFlag;    // 有无正在记录的骨架
		int kn;          // 迈步时长记录点
		int lf, rf;      // 步长记录
		int kneeFlag, footFlag, h_k, k_f; // 膝关节相关
		double sum;      // 求和临时变量

		double unit;                     // 比例单位（胸颈距离）
		double distance;                 // 其余比例临时记录
		double shoulder[NUM];            // 肩宽比例
		double hip[NUM];                 // 臀宽比例
		double hip_knee[NUM];            // 大腿比例
		double knee_foot[NUM];           // 小腿比例
		unsigned long timeRecord[NUM];   // 迈步周期（原始）
		double time[NUM];                // 迈步周期
		double leftFront[NUM];           // 步长（左脚在前）
		double rightFront[NUM];          // 步长（右脚在前）

		double record[7];  // 最终记录数据
		double compare[7]; // 读取数据库信息
	} person;

	bzero(&person, sizeof(person));

	// socket
	int sockfd;
	struct sockaddr_in servaddr;
	const char *addr = "192.168.12.199";
	char trans_f[5] = "7";
	char trans_t[5] = "8";

	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("socket error.\n");
		exit(1);
	}
	bzero(&servaddr, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(SERV_PORT);
	inet_pton(AF_INET, addr, &servaddr.sin_addr);

	if (connect(sockfd, (SA *)&servaddr, sizeof(servaddr)) < 0) {
		printf("connect error.\n");
		exit(1);
	}

	if (fileExists(SAMPLE_XML_PATH))
		fn = SAMPLE_XML_PATH;
	else if (fileExists(SAMPLE_XML_PATH_LOCAL))
		fn = SAMPLE_XML_PATH_LOCAL;
	else {
		printf("Could not find '%s' nor '%s'. Aborting.\n", SAMPLE_XML_PATH,
			SAMPLE_XML_PATH_LOCAL);
		return XN_STATUS_ERROR;
	}
	printf("Reading config from: '%s'\n", fn);

retry: nRetVal = g_Context.InitFromXmlFile(fn, g_scriptNode, &errors);
	if (nRetVal == XN_STATUS_NO_NODE_PRESENT) {
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (nRetVal);
	}
	else if (nRetVal != XN_STATUS_OK) {
		printf("Open failed: %s\n", xnGetStatusString(nRetVal));
		return (nRetVal);
	}

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}
	//	XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete,
	//			hPoseDetected;
	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		printf("Supplied user generator doesn't support skeleton\n");
		return 1;
	}
	nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser,
		NULL, hUserCallbacks);
	CHECK_RC(nRetVal, "Register to user callbacks");
	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(
		UserCalibration_CalibrationStart, NULL, hCalibrationStart);
	CHECK_RC(nRetVal, "Register to calibration start");
	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(
		UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
	CHECK_RC(nRetVal, "Register to calibration complete");

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(
			XN_CAPABILITY_POSE_DETECTION)) {
			printf("Pose required, but not supported\n");
			return 1;
		}
		nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(
			UserPose_PoseDetected, NULL, hPoseDetected);
		CHECK_RC(nRetVal, "Register to Pose Detected");
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	//	XnUserID aUsers[MAX_NUM_USERS];
	//	XnUInt16 nUsers;
	//	XnSkeletonJointTransformation joint_1, joint_2, joint_3, joint_4, joint_5,
	//			joint_6;

	printf("Starting to run\n");
	if (g_bNeedPose) {
		printf("Assume calibration pose\n");
	}
	/*****************************************************************************/
	while (!xnOSWasKeyboardHit()) {
		g_Context.WaitOneUpdateAll(g_UserGenerator);
		/* print the torso information for the first user already tracking */
		nUsers = MAX_NUM_USERS;
		g_UserGenerator.GetUsers(aUsers, nUsers);

		if (g_UserGenerator.GetSkeletonCap().IsTracking(
			aUsers[person.userNo]) != FALSE) {
			person.userFlag = 1; // 存在一套有效骨架
		}
		else {
			for (XnUInt16 i = 0; i < nUsers; i++) {
				if (g_UserGenerator.GetSkeletonCap().IsTracking(
					aUsers[i]) == FALSE)
					continue;
				person.userNo = i;
				person.userFlag = 1;
				break; // 仅识别最先找到的那个人
			}
		}

		if (person.userFlag == 1) { // 存在可识别骨架，开始记录相关信息
			person.userFlag = 0;
			// 记录内容
			g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(
				aUsers[person.userNo], XN_SKEL_TORSO, joint_1);
			//printf("user %d: [No.%d] torso at (%6.2f)\n", aUsers[person.userNo], person.n, joint_1.position.position.Z);

			if (rec == joint_1.position.position.Z)  // 判定人物已离开
				flag++;
			else
				rec = joint_1.position.position.Z;

			g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(
				aUsers[person.userNo], XN_SKEL_NECK, joint_2);
			// 以胸颈距离作为比例单位
			person.unit = 1 / PythagoreanTheorem(joint_1, joint_2);
			// 肩宽比例
			g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(
				aUsers[person.userNo], XN_SKEL_LEFT_SHOULDER, joint_1);
			g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(
				aUsers[person.userNo], XN_SKEL_RIGHT_SHOULDER, joint_2);
			person.distance = PythagoreanTheorem(joint_1, joint_2);
			person.shoulder[person.n] = person.distance * person.unit;
			// 臀宽比例
			g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(
				aUsers[person.userNo], XN_SKEL_LEFT_HIP, joint_1);
			g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(
				aUsers[person.userNo], XN_SKEL_RIGHT_HIP, joint_2);
			person.distance = PythagoreanTheorem(joint_1, joint_2);
			person.hip[person.n] = person.distance * person.unit;
			// 大小腿比例及迈步周期
			g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(
				aUsers[person.userNo], XN_SKEL_LEFT_KNEE, joint_3);
			g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(
				aUsers[person.userNo], XN_SKEL_RIGHT_KNEE, joint_4);
			g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(
				aUsers[person.userNo], XN_SKEL_LEFT_FOOT, joint_5);
			g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(
				aUsers[person.userNo], XN_SKEL_RIGHT_FOOT, joint_6);
			if (joint_3.position.position.Z < joint_4.position.position.Z
				&& person.kneeFlag == 0) { // 左膝在前
				person.kneeFlag = 1;
				person.distance = PythagoreanTheorem(joint_1, joint_3);
				person.hip_knee[person.h_k++] = person.distance * person.unit;
				person.distance = PythagoreanTheorem(joint_3, joint_5);
				person.knee_foot[person.k_f++] = person.distance * person.unit;
			}
			else if (joint_3.position.position.Z > joint_4.position.position.Z
				&& person.kneeFlag == 1) { // 右膝在前
				person.timeRecord[person.kn++] = GetTickCount();
				person.kneeFlag = 0;
				person.distance = PythagoreanTheorem(joint_2, joint_4);
				person.hip_knee[person.h_k++] = person.distance * person.unit;
				person.distance = PythagoreanTheorem(joint_4, joint_6);
				person.knee_foot[person.k_f++] = person.distance * person.unit;
			}
			// 步长
			if (joint_5.position.position.Z < joint_6.position.position.Z) {
				person.leftFront[person.lf++] = joint_6.position.position.Z
					- joint_5.position.position.Z;
				//printf("user: [No.%d] left at (%6.2f)\n", person.n, person.leftFront[person.lf-1]);
			}
			else if (joint_5.position.position.Z
			> joint_6.position.position.Z) {
				person.rightFront[person.rf++] = joint_5.position.position.Z
					- joint_6.position.position.Z;
				//printf("user: [No.%d] right at (%6.2f)\n", person.n, person.leftFront[person.rf-1]);
			}
			if (++person.n == NUM) {
				// 转换时间类型
				int flag = 0;
				for (int i = 0; i < person.kn; i++) {
					int ti = (int)person.timeRecord[i] % 100000;
					double td = (double)ti / 10000;
					if (td < 1)
						flag = 1;
					if (flag == 1)
						person.time[i] = td + 10;
					else
						person.time[i] = td;
				}
				// 平均肩宽比例
				person.sum = 0;
				for (int i = 0; i < NUM; i++) {
					person.sum += person.shoulder[i];
				}
				person.record[0] = person.sum / NUM;
				// 平均臀宽比例
				person.sum = 0;
				for (int i = 0; i < NUM; i++) {
					person.sum += person.hip[i];
				}
				person.record[1] = person.sum / NUM;
				// 平均大腿比例
				person.sum = 0;
				//person.hip_knee[0] = 0;
				InsertSort(person.hip_knee, NUM);
				for (int i = 1; i < 5; i++) {
					person.sum += person.hip_knee[NUM - i];
				}
				person.record[2] = person.sum / 4;
				// 平均小腿比例
				person.sum = 0;
				//person.knee_foot[0] = 0;
				InsertSort(person.knee_foot, NUM);
				for (int i = 1; i < 5; i++) {
					person.sum += person.knee_foot[NUM - i];
				}
				person.record[3] = person.sum / 4;
				// 平均迈步周期
				person.sum = 0;
				double avg[NUM] = { 0 };
				int j = 0;
				for (j = 0; j < person.kn - 1; j++) {
					avg[j] = (person.timeRecord[j + 1] - person.timeRecord[j]);
				}
				InsertSort(avg, NUM);
				for (int i = 1; i <= j; i++) {
					person.sum += avg[NUM - i];
				}
				if (j == 0)
					j = 1;
				person.record[4] = person.sum / j;
				// 平均步长
				person.sum = 0;
				InsertSort(person.leftFront, NUM);
				for (int i = 1; i < 6; i++) {
					person.sum += person.leftFront[NUM - i];
				}
				person.record[5] = person.sum / 5;
				person.sum = 0;
				InsertSort(person.rightFront, NUM);
				for (int i = 1; i < 6; i++) {
					person.sum += person.rightFront[NUM - i];
				}
				person.record[6] = person.sum / 5;

				for (int i = 0; i < 7; i++) {
					printf("%d : %7.3f\n", i, person.record[i]);
				}
				printf("============= finish! =============\n");

				// 对比数据库中信息
				FILE *fp;
				int n = 0;
				int result = 0;
				double a = 0;
				char arr[MAXLINE] = { 0 };
				char *p = arr;
				if ((fp = fopen("record.txt", "r")) == NULL) {
					perror("Create file failed\n");
					exit(0);
				}
				if ((n = readline(fp, arr, MAXLINE)) != 0) {
					a = atof(p);
					printf("---%7.3f---\n", a);
					if (abs(a - person.record[0]) < 0.047)
						result++;
				}
				if ((n = readline(fp, arr, MAXLINE)) != 0) {
					a = atof(p);
					printf("---%7.3f---\n", a);
					if (abs(a - person.record[1]) < 0.041)
						result++;
				}
				if ((n = readline(fp, arr, MAXLINE)) != 0) {
					a = atof(p);
					printf("---%7.3f---\n", a);
					if (abs(a - person.record[2]) < 0.314)
						result++;
				}
				if ((n = readline(fp, arr, MAXLINE)) != 0) {
					a = atof(p);
					printf("---%7.3f---\n", a);
					if (abs(a - person.record[3]) < 0.4)
						result++;
				}
				if ((n = readline(fp, arr, MAXLINE)) != 0) {
					a = atof(p);
					printf("---%7.3f---\n", a);
					if (abs(a - person.record[4]) < 224.5)
						result++;
				}
				if ((n = readline(fp, arr, MAXLINE)) != 0) {
					a = atof(p);
					printf("---%7.3f---\n", a);
					if (abs(a - person.record[5]) < 31)
						result++;
				}
				if ((n = readline(fp, arr, MAXLINE)) != 0) {
					a = atof(p);
					printf("---%7.3f---\n", a);
					if (abs(a - person.record[6]) < 77.9)
						result++;
				}
				fclose(fp);
				if (result < 5) {
					printf("报警！\n");
					write(sockfd, trans_f, strlen(trans_f));
				}
				else {
					printf("通过。\n");
					write(sockfd, trans_t, strlen(trans_t));
				}
				bzero(&person, sizeof(person));
				sleep(5);
			}
			if (flag == 5) {
				flag = 0;
				bzero(&person, sizeof(person));
				goto retry;
			}
		}
	}
	g_scriptNode.Release();
	g_UserGenerator.Release();
	g_Context.Release();
}
