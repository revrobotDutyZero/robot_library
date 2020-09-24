### a series of action, somewhat similar to what a path is now
gobot_msgs/TaskMetadata metadata

string media_path   # media path to be played during the path
                    # note that the point media will override this
int64 loop          # well, loop
                    # integer because -1 mean infinite. Is there a better way to do this?

int64 cur_loop_count

gobot_msgs/NavJob[] jobs

uint16 cur_job_idx