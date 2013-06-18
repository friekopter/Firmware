

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <systemlib/err.h>
#include <unistd.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>

#include <quat_log.h>


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static const int MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log folders */
unsigned quat_log_bytes = 0;

static const char *mountpoint = "/fs/microsd";
int sysvector_file = -1;
int mavlink_fd = -1;
static uint64_t starttime = 0;

/**
 * SD log management function.
 */
__EXPORT int quat_log_main(int argc, char *argv[]);

/**
 * Mainloop of quat log deamon.
 */
int quat_log_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static int file_exist(const char *filename);

/**
 * Print the current status.
 */
static void print_quat_log_status(void);

/**
 * Create folder for current logging session.
 */
static int create_logfolder(char *folder_path);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	errx(1, "usage: quat_log {start|stop|status} [i]\n\n");
}

/**
 * @return 0 if file exists
 */
int file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer);
}

void print_quat_log_status()
{
	unsigned bytes = quat_log_bytes;
	float mebibytes = (float)bytes / 1024.0f / 1024.0f;
	float seconds = ((float)(hrt_absolute_time() - starttime)) / 1000000.0f;

	warnx("wrote %4.2f MiB (average %5.3f MiB/s).\n", (double)mebibytes, (double)(mebibytes / seconds));
}

/**
 * The quat log deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int quat_log_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("quat_log already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("quat_log",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT - 30,
					 4096,
					 quat_log_thread_main,
					 (const char **)argv);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (!thread_running) {
			printf("\tquat_log is not started\n");
		}

		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			print_quat_log_status();

		} else {
			printf("\tquat_log not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int create_logfolder(char *folder_path)
{
	/* make folder on sdcard */
	uint16_t foldernumber = 1; // start with folder 0001
	int mkdir_ret;

	/* look for the next folder that does not exist */
	while (foldernumber < MAX_NO_LOGFOLDER) {
		/* set up file path: e.g. /mnt/sdcard/sensorfile0001.log */
		sprintf(folder_path, "%s/session%04u", mountpoint, foldernumber);
		mkdir_ret = mkdir(folder_path, S_IRWXU | S_IRWXG | S_IRWXO);
		/* the result is -1 if the folder exists */

		if (mkdir_ret == 0) {
			/* folder does not exist, success */
			break;

		} else if (mkdir_ret == -1) {
			/* folder exists already */
			foldernumber++;
			continue;

		} else {
			warn("failed creating new folder");
			return -1;
		}
	}

	if (foldernumber >= MAX_NO_LOGFOLDER) {
		/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
		warn("all %d possible folders exist already", MAX_NO_LOGFOLDER);
		return -1;
	}

	return 0;
}

int quat_log_thread_main(int argc, char *argv[])
{
	/* log every n'th value (skip three per default) */
	unsigned intervall = 5;
    uint32_t loops = 0;

	/* work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;
	int ch;

	while ((ch = getopt(argc, argv, "r:i")) != EOF) {
		switch (ch) {
		case 'i':
			{
				/* set logging interval in ms */
				unsigned i = strtoul(optarg, NULL, 10);

				if (i <= 0 || i > 2000) {
					errx(1, "Wrong skip value of %d, out of range (1..2000)\n", i);
				} else {
					intervall = i;
				}
			}
			break;

		case '?':
			if (optopt == 'c') {
				warnx("Option -%c requires an argument.\n", optopt);
			} else if (isprint(optopt)) {
				warnx("Unknown option `-%c'.\n", optopt);
			} else {
				warnx("Unknown option character `\\x%x'.\n", optopt);
			}
			break;

		default:
			usage("unrecognized flag");
			errx(1, "exiting.");
			break;
		}
	}

	if (file_exist(mountpoint) != OK) {
		errx(1, "logging mount point %s not present, exiting.", mountpoint);
	}

	char folder_path[64];

	if (create_logfolder(folder_path))
		errx(1, "unable to create logging folder, exiting.");

	FILE *logging_file;
	//int logging_file = -1;

	/* string to hold the path to the sensorfile */
	char path_buf[64] = "";

	/* only print logging path, important to find log file later */
	warnx("logging to directory %s\n", folder_path);


	/* set up file path: e.g. /mnt/sdcard/session0001/blackbox.txt */
	sprintf(path_buf, "%s/%s.log", folder_path, "sensor_log");
/*
	if (0 == (logging_file = open(path_buf, O_CREAT | O_WRONLY | O_DSYNC))) {
		errx(1, "opening %s failed.\n", path_buf);
	}*/

	if (NULL == (logging_file = fopen(path_buf, "w"))) {
		errx(1, "opening %s failed.\n", path_buf);
		return 1;
	}

	int logging_file_no = fileno(logging_file);

	/* --- IMPORTANT: DEFINE NUMBER OF ORB STRUCTS TO WAIT FOR HERE --- */
	/* number of messages */
	const ssize_t fdsc = 5;
	/* Sanity check variable and index */
	ssize_t fdsc_count = 0;
	/* file descriptors to wait for */
	struct pollfd fds[fdsc];

	struct {
		struct gyro_report gyro_report;
		struct mag_report mag_report;
		struct battery_status_s battery_status;
		struct accel_report accel_report;
		struct baro_report barometer;
	} buf;
	memset(&buf, 0, sizeof(buf));

	struct {
		int gyro_sub;
		int mag_sub;
		int battery_sub;
		int accel_sub;
		int baro_sub;
	} subs;

	subs.gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	fds[fdsc_count].fd = subs.gyro_sub;
	fds[fdsc_count].events = POLLIN;
	orb_set_interval(subs.gyro_sub, intervall);
	fdsc_count++;

	subs.mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	fds[fdsc_count].fd = subs.mag_sub;
	fds[fdsc_count].events = POLLIN;
	//orb_set_interval(subs.mag_sub, intervall);
	fdsc_count++;

	subs.battery_sub = orb_subscribe(ORB_ID(battery_status));
	fds[fdsc_count].fd = subs.battery_sub;
	fds[fdsc_count].events = POLLIN;
	//orb_set_interval(subs.battery_sub, intervall);
	fdsc_count++;

	subs.accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	fds[fdsc_count].fd = subs.accel_sub;
	fds[fdsc_count].events = POLLIN;
	//orb_set_interval(subs.accel_sub, intervall);
	fdsc_count++;

	subs.baro_sub = orb_subscribe(ORB_ID(sensor_baro));
	fds[fdsc_count].fd = subs.baro_sub;
	fds[fdsc_count].events = POLLIN;
	//orb_set_interval(subs.baro_sub, intervall);
	fdsc_count++;

	/* WARNING: If you get the error message below,
	 * then the number of registered messages (fdsc)
	 * differs from the number of messages in the above list.
	 */
	if (fdsc_count > fdsc) {
		warn("WARNING: Not enough space for poll fds allocated. Check %s:%d.\n", __FILE__, __LINE__);
		fdsc_count = fdsc;
	}

	perf_counter_t quat_log_do_perf = perf_alloc(PC_ELAPSED, "quat_log_do");
	perf_counter_t quat_log_write_perf = perf_alloc(PC_ELAPSED, "quat_log_write");
	perf_counter_t quat_log_sync_perf = perf_alloc(PC_ELAPSED, "quat_log_sync");

	logInit(&buf.gyro_report, &buf.mag_report, &buf.battery_status, &buf.accel_report, &buf.barometer);

	thread_running = true;

	starttime = hrt_absolute_time();
	printf("[quat log] initialized, packet size: %d\n",logData.packetSize);

	while (!thread_should_exit) {

		// Only poll for gyro
		int poll_ret = poll(fds, 1, 3000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* XXX this means none of our providers is giving us data - might be an error? */
		} else if (poll_ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else {

			int ifds = 0;

			if (fds[ifds++].revents & POLLIN) {

				perf_begin(quat_log_do_perf);
				/* copy command into local buffer */
				orb_copy(ORB_ID(sensor_gyro), subs.gyro_sub, &buf.gyro_report);
				orb_copy(ORB_ID(sensor_mag), subs.mag_sub, &buf.mag_report);
				orb_copy(ORB_ID(battery_status), subs.battery_sub, &buf.battery_status);
				orb_copy(ORB_ID(sensor_accel), subs.accel_sub, &buf.accel_report);
				orb_copy(ORB_ID(sensor_baro), subs.baro_sub, &buf.barometer);
				if (!(loops % 200)) {
				    logDoHeader();
				}
				logDo();
				perf_end(quat_log_do_perf);
				perf_begin(quat_log_write_perf);
				size_t writtenBytes = logWrite(logging_file);
				quat_log_bytes += writtenBytes;
				perf_end(quat_log_write_perf);
				// quat_log_bytes += fprintf(logging_file,"%s\n",logData.logBuf);
				if (loops % 100 == 0) {
					perf_begin(quat_log_sync_perf);
					fsync(logging_file_no);
					perf_end(quat_log_sync_perf);
					static uint64_t lastTime = 0;
					uint64_t currentTime = hrt_absolute_time();
					float diff = (float)(currentTime - lastTime)/1000.0f;
					unsigned bytes = quat_log_bytes;
					float mebibytes = (float)bytes / 1024.0f / 1024.0f;
					if(mebibytes > 300.0f) thread_should_exit = true;
					printf("[quat log] %8.4fms\t written: %dbytes\ttotal: %8.4fMegabytes\ttemp: %8.4fC\n",
							diff, writtenBytes, mebibytes, buf.gyro_report.temperature);
					lastTime = currentTime;
				}
			}
		}
		loops++;
	}

	print_quat_log_status();
	warnx("[quat log] exiting.\n\n");
	fsync(logging_file_no);
	fclose(logging_file);
	thread_running = false;

	return 0;
}

