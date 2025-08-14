source("single_robot.R") # Will import and run to collect data

participant_id <- "17"
#robot_filename <- "04-26-2024_16-36-06-928.csv"

log_data <- get_participant_log_data()
single_robot <- get_robot_movement_data(participant_id, get_participant_robot_time_offset(participant_id, log_data), acc_crossing_noise_threshold, movement_classification_acceleration_threshold)
movement_blocks <- get_movement_blocks(single_robot)
progress_tracker_data <- get_progress_tracker_data(participant_id, single_robot)
single_garmin <- get_garmin_data(participant_id, min(progress_tracker_data$Start_Timestamp_r, na.rm=TRUE), max(progress_tracker_data$End_Timestamp_r, na.rm=TRUE))
empatica_data <- get_empatica_data(participant_id)
single_garmin <- add_lap_data_to_biometric(participant_id, single_garmin, progress_tracker_data)
single_empatica <- get_empatica_data(participant_id)
single_empatica_hr <- single_empatica[1][[1]]
single_empatica_eda <- single_empatica[2][[1]]
single_empatica_temp <- single_empatica[3][[1]]
single_empatica_bvp <- single_empatica[4][[1]]

plot_start_time <- "05-06-2024_13:51:40.000" # Hour:Minute:Second (Example: 15:58:00)
plot_end_time <- "05-06-2024_13:52:10.000"
plot_end_time <- NA # Comment out for custom X-axis timespan

plot_color_data_raw <- "brown" #"#8A8FDB"
#plot_color_data_smooth <- "#DBA38A"
plot_color_data_smooth <- "#D81B60"
plot_raw_alpha <- 0.1
plot_raw_width <- 0.2
plot_smooth_alpha <- 1.0
plot_smooth_width <- 0.4

plot_laps_alpha <- 0.15

plot_color_zero_crossing <- "#A2E084"
plot_zero_crossing_alpha <- 0.25
plot_zero_crossing_width <- 1.0

plot_color_accelerating <- "#b0e3e8"
plot_color_slowing <- "#e8b4b0"
plot_color_none <- "white"

plot_color_forward <- "#1E88E5"
plot_color_backward <- "#FFC107"
plot_color_hybrid <- "#004D40"

##### Plot

## Timespan
display_timespan <- NA
if(!is.na(plot_start_time) & !is.na(plot_end_time))
{
  display_timespan <- as.POSIXct(strptime(c(plot_start_time, plot_end_time), format = "%m-%d-%Y_%H:%M:%OS"))
}
get_display_timespan <- function(){
  #if(is.na(display_timespan))
  if(typeof(display_timespan) == "logical")
  {
    return(NULL)
  }
  else
  {
    return(display_timespan)
  }
}

## Set Lap names
progress_tracker_data[progress_tracker_data$Route == "Combined",]$Route <- "Hybrid" # Rename
progress_tracker_data$Route <- factor(progress_tracker_data$Route, levels=c("Forward", "Backward", "Hybrid"), labels=c("Forward", "Backward", "Hybrid")) # Refactor


## Plots

plot_dist <- ggplot(single_robot, aes(x=as.POSIXct(timestamp_r), y=pos)) +
  geom_line(group=1)
#print(plot_dist)

plot_height <- ggplot() +
  geom_rect(data=progress_tracker_data,
            aes(xmin=as.POSIXct(Start_Timestamp_r),
                xmax=as.POSIXct(End_Timestamp_r),
                ymin=-Inf,
                ymax=Inf,
                fill=Route),
            alpha=plot_laps_alpha) +
  geom_line(data=single_robot, aes(x=as.POSIXct(timestamp_r), y=pos_z_vision_body),
            group=1,
            color=plot_color_data_smooth,
            alpha=plot_smooth_alpha,
            size=plot_smooth_width) +
  # stat_smooth(data=single_robot, aes(x=as.POSIXct(timestamp_r), y=pos_z_vision_body),
  #             geom="line",
  #             method="loess",
  #             span=loess_smoothing_vel,
  #             color=plot_color_data_smooth,
  #             alpha=plot_smooth_alpha,
  #             linewidth=plot_smooth_width) + 
  scale_x_datetime(limits=get_display_timespan(),
                   date_labels="%H:%M:%S",
                   expand=c(0,0)) +
  scale_fill_manual(values = c("Forward" = plot_color_forward, "Backward" = plot_color_backward, "Hybrid" = plot_color_hybrid)) +
  #scale_y_continuous(limits=c(0, 2.0)) +
  xlab("") +
  ylab("Height") +
  theme_bw() +
  theme()
#print(plot_height)

plot_vel <- ggplot() +
  geom_rect(data=progress_tracker_data,
            aes(xmin=as.POSIXct(Start_Timestamp_r),
                xmax=as.POSIXct(End_Timestamp_r),
                ymin=-Inf,
                ymax=Inf,
                fill=Route),
            alpha=plot_laps_alpha) +
  # geom_vline(data=subset(movement_blocks),
  #            aes(xintercept=as.POSIXct(timestamp_r)),
  #            color=plot_color_zero_crossing,
  #            alpha=plot_zero_crossing_alpha,
  #            size=plot_zero_crossing_width) +
  geom_line(data=single_robot, aes(x=as.POSIXct(timestamp_r), y=vel_linear),
            group=1,
            color=plot_color_data_raw,
            alpha=plot_raw_alpha,
            size=plot_raw_width) +
  geom_line(data=single_robot, aes(x=as.POSIXct(timestamp_r), y=vel_linear_loess),
            group=1,
            color=plot_color_data_smooth,
            alpha=plot_smooth_alpha,
            linewidth=plot_smooth_width) +
  # geom_text(data=movement_blocks, aes(as.POSIXct(timestamp_r), label=as.character(movement_number)),
  #           y=1.75,
  #           check_overlap=TRUE) +
  scale_x_datetime(limits=get_display_timespan(),
                   date_labels="%H:%M:%S",
                   expand=c(0,0)) +
  scale_fill_manual(values = c("Forward" = plot_color_forward, "Backward" = plot_color_backward, "Hybrid" = plot_color_hybrid)) +
  scale_y_continuous(limits=c(0, 2.0)) +
  xlab("") +
  ylab(expression(paste("Velocity (", m/s, ")"))) + 
  theme_bw() +
  theme()
#print(plot_vel)

plot_acc <- ggplot() +
  geom_rect(data=progress_tracker_data,
            aes(xmin=as.POSIXct(Start_Timestamp_r),
                xmax=as.POSIXct(End_Timestamp_r),
                ymin=-Inf,
                ymax=Inf,
                fill=Route),
            alpha=plot_laps_alpha) +
  # geom_tile(aes(y=0, fill=movement_classification,height=8,alpha=0.1)) +
  # geom_rect(aes(xmin=as.POSIXct(timestamp_r), xmax=as.POSIXct(timestamp_r),
  #               ymin=-Inf, ymax=Inf,
  #               color=movement_classification),
  #           size=0.7,
  #           alpha=1) +
  # geom_vline(data=subset(movement_blocks),
  #            aes(xintercept=as.POSIXct(timestamp_r)),
  #            color=plot_color_zero_crossing,
  #            alpha=plot_zero_crossing_alpha,
  #            size=plot_zero_crossing_width) +
  geom_line(data=single_robot, aes(x=as.POSIXct(timestamp_r), y=acc_linear),
            group=1,
            color=plot_color_data_raw,
            alpha=plot_raw_alpha,
            size=plot_raw_width) +
  geom_line(data=single_robot, aes(x=as.POSIXct(timestamp_r), y=acc_linear_loess),
            group=1,
            color=plot_color_data_smooth,
            alpha=plot_smooth_alpha,
            linewidth=plot_smooth_width) +
  # geom_vline(data=subset(single_robot, acc_zero_crossing==TRUE),
  #           aes(xintercept=as.POSIXct(timestamp_r)),
  #           color=plot_color_zero_crossing,
  #           alpha=plot_zero_crossing_alpha) +
  # geom_text(data=movement_blocks, aes(as.POSIXct(timestamp_r), label=as.character(movement_number)),
  #            y=3,
  #           check_overlap=TRUE) + 
  scale_x_datetime(limits=get_display_timespan(),
                   date_labels="%H:%M:%S",
                   expand=c(0,0)) +
  scale_y_continuous(limits=c(-4.0, 4.0)) +
  # scale_y_continuous(limits=c(-1,1),
  #                    expand=c(0,0)) +
  scale_color_manual(values=c(Accelerating=plot_color_accelerating, Slowing=plot_color_slowing, None=plot_color_none)) +
  scale_fill_manual(values = c("Forward" = plot_color_forward, "Backward" = plot_color_backward, "Hybrid" = plot_color_hybrid)) +
  xlab("") +
  ylab(expression(paste("Acceleration (", m/s^2, ")"))) + 
  theme_bw() +
  theme()
#print(plot_acc)

plot_jer <- ggplot() +
  geom_rect(data=progress_tracker_data,
            aes(xmin=as.POSIXct(Start_Timestamp_r),
                xmax=as.POSIXct(End_Timestamp_r),
                ymin=-Inf,
                ymax=Inf,
                fill=Route),
            alpha=plot_laps_alpha) +
  # geom_vline(data=subset(movement_blocks),
  #            aes(xintercept=as.POSIXct(timestamp_r)),
  #            color=plot_color_zero_crossing,
  #            alpha=plot_zero_crossing_alpha,
  #            size=plot_zero_crossing_width) +
  geom_line(data=single_robot, aes(x=as.POSIXct(timestamp_r), y=jer_linear),
            group=1,
            color=plot_color_data_raw,
            alpha=plot_raw_alpha,
            size=plot_raw_width) +
  geom_line(data=single_robot, aes(x=as.POSIXct(timestamp_r), y=jer_linear_loess),
            group=1,
            color=plot_color_data_smooth,
            alpha=plot_smooth_alpha,
            linewidth=plot_smooth_width) +
  # geom_text(data=movement_blocks, aes(as.POSIXct(timestamp_r), label=as.character(movement_number)),
  #           y=15,
  #           check_overlap=TRUE) + 
  scale_x_datetime(limits=get_display_timespan(),
                   date_labels="%H:%M:%S",
                   expand=c(0,0)) +
  scale_y_continuous(limits=c(-20.0, 20.0)) +
  scale_fill_manual(values = c("Forward" = plot_color_forward, "Backward" = plot_color_backward, "Hybrid" = plot_color_hybrid)) +
  xlab("") + 
  ylab(expression(paste("Jerk (", m/s^3, ")"))) + 
  theme_bw() +
  theme()
#print(plot_jer)

#combined_plot <- ggarrange(plot_vel, plot_acc, nrow = 2)
combined_plot <- ggarrange(plot_height, plot_vel, plot_acc, plot_jer,
                           ncol = 1,
                           top = paste0("Participant ID: ", participant_id))

plot_laps <- ggplot() +
  geom_rect(data=progress_tracker_data,
            aes(xmin=as.POSIXct(Start_Timestamp_r),
                xmax=as.POSIXct(End_Timestamp_r),
                ymin=-Inf,
                ymax=Inf,
                fill=Route),
            alpha=plot_laps_alpha) +
  geom_line(data=single_robot,
            aes(x=as.POSIXct(timestamp_r), y=jer_linear_loess),
            group=1,
            color=plot_color_data_smooth,
            alpha=plot_smooth_alpha,
            linewidth=plot_smooth_width) +
  scale_x_datetime(limits=get_display_timespan(),
                   date_labels="%H:%M:%S",
                   expand=c(0,0)) +
  theme_bw() +
  theme()  
#print(plot_laps)

plot_hr <- ggplot() +
  geom_hline(yintercept = single_garmin$resting_heart_rate[1],
             color="black",
             alpha=0.5,
             size=1.0) +
  geom_rect(data=progress_tracker_data,
            aes(xmin=as.POSIXct(Start_Timestamp_r),
                xmax=as.POSIXct(End_Timestamp_r),
                ymin=-Inf,
                ymax=Inf,
                fill=Route),
            alpha=plot_laps_alpha) +
  geom_line(data=single_garmin,
            aes(x=as.POSIXct(timestamp_r), y=heart_rate),
            group=1,
            color=plot_color_data_smooth,
            alpha=1.0,
            size=1.0) +
  # geom_line(data=single_empatica_hr,
  #           aes(x=as.POSIXct(timestamp_r), y=bpm),
  #           group=1,
  #           color="red",
  #           alpha=1.0,
  #           size=1.0) +
  scale_x_datetime(limits=get_display_timespan(),
                   date_labels="%H:%M:%S",
                   expand=c(0,0)) +
  #scale_y_continuous(limits=c(20, 200)) +
  scale_fill_manual(values = c("Forward" = plot_color_forward, "Backward" = plot_color_backward, "Hybrid" = plot_color_hybrid)) +
  xlab("") +
  ylab("Heart Rate (BPM)") +
  theme_bw() +
  theme()
#print(plot_hr)

plot_eda <- ggplot() +
  geom_rect(data=progress_tracker_data,
            aes(xmin=as.POSIXct(Start_Timestamp_r),
                xmax=as.POSIXct(End_Timestamp_r),
                ymin=-Inf,
                ymax=Inf,
                fill=Route),
            alpha=plot_laps_alpha) +
  geom_line(data=single_empatica_eda,
            aes(x=as.POSIXct(timestamp_r), y=eda),
            group=1,
            color=plot_color_data_smooth,
            alpha=1.0,
            size=1.0) +
  scale_x_datetime(limits=get_display_timespan(),
                   date_labels="%H:%M:%S",
                   expand=c(0,0)) +
  #scale_y_continuous(limits=c(20, 200)) +
  scale_fill_manual(values = c("Forward" = plot_color_forward, "Backward" = plot_color_backward, "Hybrid" = plot_color_hybrid)) +
  xlab("") +
  ylab("EDA") +
  theme_bw() +
  theme()
#print(plot_eda)

plot_temp <- ggplot() +
  geom_rect(data=progress_tracker_data,
            aes(xmin=as.POSIXct(Start_Timestamp_r),
                xmax=as.POSIXct(End_Timestamp_r),
                ymin=-Inf,
                ymax=Inf,
                fill=Route),
            alpha=plot_laps_alpha) +
  geom_line(data=single_empatica_temp,
            aes(x=as.POSIXct(timestamp_r), y=temp),
            group=1,
            color=plot_color_data_smooth,
            alpha=1.0,
            size=1.0) +
  scale_x_datetime(limits=get_display_timespan(),
                   date_labels="%H:%M:%S",
                   expand=c(0,0)) +
  #scale_y_continuous(limits=c(20, 200)) +
  scale_fill_manual(values = c("Forward" = plot_color_forward, "Backward" = plot_color_backward, "Hybrid" = plot_color_hybrid)) +
  xlab("") +
  ylab("Temperature (C)") +
  theme_bw() +
  theme()
#print(plot_temp)

plot_bvp <- ggplot() +
  geom_rect(data=progress_tracker_data,
            aes(xmin=as.POSIXct(Start_Timestamp_r),
                xmax=as.POSIXct(End_Timestamp_r),
                ymin=-Inf,
                ymax=Inf,
                fill=Route),
            alpha=plot_laps_alpha) +
  geom_line(data=single_empatica_bvp,
            aes(x=as.POSIXct(timestamp_r), y=bvp),
            group=1,
            color=plot_color_data_smooth,
            alpha=1.0,
            size=1.0) +
  scale_x_datetime(limits=get_display_timespan(),
                   date_labels="%H:%M:%S",
                   expand=c(0,0)) +
  #scale_y_continuous(limits=c(20, 200)) +
  scale_fill_manual(values = c("Forward" = plot_color_forward, "Backward" = plot_color_backward, "Hybrid" = plot_color_hybrid)) +
  xlab("") +
  ylab("BVP") +
  theme_bw() +
  theme()
#print(plot_bvp)

combined_plot <- ggarrange(plot_hr, plot_eda, plot_temp, plot_bvp,
                           ncol = 1,
                           top = paste0("Participant ID: ", participant_id))

