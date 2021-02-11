// Copyright 2016-2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/**
 * File: tracer.cpp
 * Author: Jan Feitsma
 */



//
//
//// thread safety
//namespace
//{
//   boost::mutex g_trace_mutex;
//}
//
//#define BUFFER_SIZE 65536
//
////
////
//void Tracer::reset()
//{
//    m_enabled = false;
//    m_lastcheck = 0;
//    sprintf(m_folder, "/var/tmp");
//    m_filename_template = "%s/trace_%c%d_%s.txt"; // folder, then robot id (e.g A1 or B6), then processId (e.g. visionNode).
//    m_filename_template2 = "%s/trace_%c%d_%s_%d.txt"; // as above but only when restarting software - to avoid files being overwritten
//    m_bucket = 0;
//    m_num_buckets = NUM_BUCKETS;
//    m_bucket_lines = BUCKET_LINES;
//    m_bucket_size = BUCKET_SIZE;
//    m_current_file = NULL;
//    m_num_written_lines = 0;
//    m_num_written_bytes = 0;
//    m_thread_ids.clear();
//}
//
//bool Tracer::enabled()
//{
//    // check sometimes if trigger file has appeared or disappeared
//    double elapsed = getTimeNow() - m_lastcheck;
//    if (elapsed > 0.2)
//    {
//        // do the check
//        FILE *fp_trigger = NULL;
//        fp_trigger = fopen(TRACING_TRIGGER_FILE, "r");
//        if (fp_trigger)
//        {
//            // use folder from the trigger file, default back to /var/tmp
//            if (fscanf(fp_trigger, "%s", m_folder) && !strlen(m_folder))
//            {
//                sprintf(m_folder, "/var/tmp");
//            }
//            m_enabled = true;
//            fclose(fp_trigger);
//        }
//        else
//        {
//            m_enabled = false;
//        }
//        m_lastcheck = getTimeNow();
//    }
//    // restore cached result
//    return m_enabled;
//}
//
//void Tracer::setPrefix(std::string const &prefix)
//{
//    m_filename_template = "%s/" + prefix + "_%c%d_%s.txt"; // folder, then robot id (e.g A1 or B6), then processId (e.g. visionNode).
//    m_filename_template2 = "%s/" + prefix + "_%c%d_%s_%d.txt"; // as above but only when restarting software - to avoid files being overwritten
//}
//
//Tracer *Tracer::instance(std::string const &prefix)
//{
//    if (!s_instances.count(prefix))
//    {
//        s_instances[prefix] = new Tracer;
//    }
//    return s_instances[prefix];
//}
//
//std::string eventToString(falcons::eventType event)
//{
//    char buf[BUFFER_SIZE] = {0};
//	std::string tstamp = timeToString(event.timeStamp);
//	std::string thread_id = toStr(boost::this_thread::get_id());
//	if (0 > snprintf(buf, BUFFER_SIZE-1, "%s %s:%04d %s %s %s\n",
//        tstamp.c_str(),
//        event.fileName.c_str(),
//        event.lineNumber,
//        event.funcName.c_str(),
//        thread_id.c_str(),
//        event.message.c_str()))
//    {
//        // avoid buffer overflow, truncate string
//        buf[BUFFER_SIZE-1] = 0;
//    }
//    return buf;
//}
//
//void trace_write_event(falcons::eventType const &event)
//{
//	trace_write_str(eventToString(event), "trace");
//}
//
//void trace_write_str(std::string const &msg, std::string const &prefix)
//{
//	char buf[BUFFER_SIZE] = {0};
//	static double _lastTimestamp = getTimeNow();
//	FILE * tmp_fp;
//	// lock for parallel use
//	boost::mutex::scoped_lock l(g_trace_mutex);
//	// get Tracer instance
//	Tracer *t_inst = Tracer::instance(prefix);
//	t_inst->setPrefix(prefix);
//	// exit if disabled
//	if (!t_inst->enabled()) return;
//	// open file first time
//	if (NULL == t_inst->m_current_file)
//	{
//		sprintf(buf, t_inst->m_filename_template.c_str(), t_inst->m_folder, getTeamChar(), getRobotNumber(), getProcessId().c_str());
//		printf("tracer.hpp: writing to %s\n", buf);
//		tmp_fp = fopen(buf, "r");
//		int counter = 1;
//		while (tmp_fp)
//		{
//		    fclose(tmp_fp);
//		    ++counter;
//    		sprintf(buf, t_inst->m_filename_template2.c_str(), t_inst->m_folder, getTeamChar(), getRobotNumber(), getProcessId().c_str(), counter);
//    		tmp_fp = fopen(buf, "r");
//		}
//		// file in 'buf' does not exist so open for writing
//		t_inst->m_current_file = fopen(buf, "w");
//	}
//	// write the message to file
//    fprintf(t_inst->m_current_file, "%s", msg.c_str());
//    // every once in a while, fflush to disk, so we minimize amount of tracing lost due to buffering
//    double tcurr = getTimeNow();
//    double diff = tcurr - _lastTimestamp;
//    double threshold = 0.5;
//    if (diff >= threshold)
//    {
//    	fflush(t_inst->m_current_file);
//    	_lastTimestamp = tcurr;
//    }
//	t_inst->m_num_written_lines++;
//}


