library(ggplot2)
library(egg) # Display multiple graphs with consistent panel size
library(dplyr)
options(digits.secs=3)

##### Parameters

participant_id <- "5"
#filename <- "04-26-2024_16-36-06-928.csv"

#loess_smoothing <- 0.01
loess_smoothing_vel <- 0.01
loess_smoothing_acc <- 0.0025 # 0.01
loess_smoothing_jer <- loess_smoothing_acc
acc_crossing_noise_threshold <- 0.1 # 0.001
movement_classification_acceleration_threshold <- 0.01

get_participant_log_data <- function()
{
  log_path <- file.path("..", "Subjects", "Log.csv")
  if(!file.exists(log_path))
  {
    print(paste0("Log file unable to be found at path:", log_path))
    return(NA)
  }
  log <- read.csv(log_path,
                  header=TRUE,
                  skip=2,
                  sep=",",
                  na.strings = c("", "NA"))
  
  ## Cast data
  log$SpotTimeOffset <- as.numeric(log$SpotTimeOffset)
  
  return(log)
}

get_participant_robot_time_offset <- function(participant_id, log_data)
{
  participant_row <- subset(log_data, Participant.ID==participant_id)
  if(nrow(participant_row) == 0)
  {
    print(paste0("No time offset data present in log for participant ", participant_id))
    return(0)
  }
  else if(nrow(participant_row) > 1)
  {
    print(paste0("Multiple rows for participant ", participant_id, " found in log"))
    return(0)    
  }
  return(participant_row$SpotTimeOffset)
}

get_robot_movement_data <- function(participant_id, robot_time_offset, acc_crossing_noise_threshold, movement_classification_acceleration_threshold)
{
  print(paste0("Grabbing movement data for participant: ", participant_id))
  
  ##### Determine filename
  #full_path <- file.path("..", "Subjects", participant_id, filename)
  full_path <- NA
  subject_dir <- file.path("..", "Subjects", participant_id)
  for(file in (list.files(subject_dir, full.names=TRUE, recursive=FALSE)))
  {
    if(grepl(".csv", file, fixed = TRUE)) # Will need to update this methodology if more than one .csv file are in this subject's base directory. Review later...
    {
      full_path <- file
    }
  }
  print(paste0("Parsing movement file: ", full_path))
  
  ##### Read data
  single_data <- read.csv(full_path,
                        header=TRUE,
                        sep=",",
                        na.strings = c("", "NA"))
  if(!class(single_data) == "data.frame") # Movement data missing
  {
    return(NA)
  }
  single_data <- subset(single_data, select = -c(X)) # Drop useless X column
  single_data <- single_data[!duplicated(single_data$timestamp_utc),] # Remove duplicate rows (very rare)
  #single_data <- distinct(single_data) # Remove duplicate rows (very rare)
  
  ##### Convert timestamps
  
  ## Fix error in recorded timestamp (timestamp recognition in R needs seconds as 'x.xxx' and not 'x-xxx')
  for(row in 1:nrow(single_data))
  {
    len <- nchar(single_data$timestamp_utc[row])
    substr(single_data$timestamp_utc[row], len-3, len-3) <- "."
  }
  
  ## Convert to R Timstamp
  single_data$timestamp_r <- as.POSIXct(single_data$timestamp_utc, format="%m-%d-%Y_%H-%M-%OS", tz="UTC", usetz=TRUE)
  single_data$timestamp_r <- format(single_data$timestamp_r, tz="America/New_York", usetz=TRUE) # Adjust timezone
  #single_data$timestamp_r <- as.POSIXct(single_data$timestamp_r) - 23.0 # Robot is about 23 seconds ahead robot_time_offset
  single_data$timestamp_r <- as.POSIXct(single_data$timestamp_r) - robot_time_offset # Account for robot time offset by reading in manually recorded information from the log file.
  
  ##### Sort and tag index
  single_data <- single_data[order(single_data$timestamp_r),]
  single_data$index <- 1:nrow(single_data)
  
  ##### Interpret accelerations
  single_data$acc_linear_x_body <- NA
  single_data$acc_linear_y_body <- NA
  single_data$acc_linear_z_body <- NA
  previous_timestamp <- NA
  previous_velocity <- c(NA, NA, NA)
  for(row in 1:nrow(single_data))
  {
    current_velocity <- c(single_data$vel_linear_x_body[row], single_data$vel_linear_y_body[row], single_data$vel_linear_z_body[row])
    current_timestamp <- single_data$timestamp_r[row]
    if(!is.na(previous_velocity[1]))
    {
      change_in_time <- as.numeric(difftime(current_timestamp, previous_timestamp), units = "secs")
      single_data$acc_linear_x_body[row] <- (current_velocity[1] - previous_velocity[1]) / change_in_time
      single_data$acc_linear_y_body[row] <- (current_velocity[2] - previous_velocity[2]) / change_in_time
      single_data$acc_linear_z_body[row] <- (current_velocity[3] - previous_velocity[3]) / change_in_time
    }
    previous_timestamp <- current_timestamp
    previous_velocity <- current_velocity
  }
  rm(previous_timestamp)
  rm(previous_velocity)
  rm(current_velocity)
  rm(current_timestamp)
  
  
  ##### Interpret jerk
  single_data$jer_linear_x_body <- NA
  single_data$jer_linear_y_body <- NA
  single_data$jer_linear_z_body <- NA
  previous_timestamp <- NA
  previous_acceleration <- c(NA, NA, NA)
  for(row in 1:nrow(single_data))
  {
    current_acceleration <- c(single_data$acc_linear_x_body[row], single_data$acc_linear_y_body[row], single_data$acc_linear_z_body[row])
    current_timestamp <- single_data$timestamp_r[row]
    if(!is.na(previous_acceleration[1]))
    {
      change_in_time <- as.numeric(difftime(current_timestamp, previous_timestamp), units = "secs")
      single_data$jer_linear_x_body[row] <- (current_acceleration[1] - previous_acceleration[1]) / change_in_time
      single_data$jer_linear_y_body[row] <- (current_acceleration[2] - previous_acceleration[2]) / change_in_time
      single_data$jer_linear_z_body[row] <- (current_acceleration[3] - previous_acceleration[3]) / change_in_time
    }
    previous_timestamp <- current_timestamp
    previous_acceleration <- current_acceleration
  }
  rm(previous_timestamp)
  rm(previous_acceleration)
  rm(current_acceleration)
  rm(current_timestamp)
  
  ##### Calculate sums and derived values
  
  ## Total position (distance from origin)
  single_data$pos <- sqrt(single_data$pos_x_vision_body^2 + single_data$pos_y_vision_body^2 + single_data$pos_z_vision_body^2)
  
  ## Total velocity
  ind <- 1:nrow(single_data)
  single_data$vel_linear <- sqrt(single_data$vel_linear_x_body^2 + single_data$vel_linear_y_body^2 + single_data$vel_linear_z_body^2)
  single_data$vel_linear_loess <- loess(single_data$vel_linear ~ ind, span=loess_smoothing_vel)$fitted
  single_data$vel_linear_loess <- pmax(single_data$vel_linear_loess, 0) # Clamp to remove negative values in the fitted data
  
  ## Total acceleration
  single_data$acc_linear <- NA
  previous_timestamp <- NA
  previous_velocity <- NA
  for(row in 1:nrow(single_data))
  {
    current_velocity <- single_data$vel_linear[row]
    current_timestamp <- single_data$timestamp_r[row]
    if(!is.na(previous_velocity))
    {
      change_in_time <- as.numeric(difftime(current_timestamp, previous_timestamp), units = "secs")
      single_data$acc_linear[row] <- (current_velocity[1] - previous_velocity[1]) / change_in_time
    }
    previous_timestamp <- current_timestamp
    previous_velocity <- current_velocity
  }
  rm(previous_timestamp)
  rm(previous_velocity)
  rm(current_velocity)
  rm(current_timestamp)
  single_data$acc_linear[is.na(single_data$acc_linear)] <- 0
  single_data$acc_linear_loess <- loess(single_data$acc_linear ~ ind, span=loess_smoothing_acc)$fitted
  # single_data$acc_linear <- sqrt(single_data$acc_linear_x_body^2 + single_data$acc_linear_y_body^2 + single_data$acc_linear_z_body^2)
  # single_data$acc_linear[is.na(single_data$acc_linear)] <- 0 # Loess gets mad at NA so replace it with 0
  # single_data$acc_linear_loess <- loess(single_data$acc_linear ~ ind, span=loess_smoothing_acc)$fitted

  ## Total jerk
  #single_data$jer_linear <- sqrt(single_data$jer_linear_x_body^2 + single_data$jer_linear_y_body^2 + single_data$jer_linear_z_body^2)
  #single_data$jer_linear[is.na(single_data$jer_linear)] <- 0
  single_data$jer_linear <- NA
  previous_timestamp <- NA
  previous_acceleration <- NA
  for(row in 1:nrow(single_data))
  {
    current_acceleration <- single_data$acc_linear[row]
    current_timestamp <- single_data$timestamp_r[row]
    if(!is.na(previous_acceleration))
    {
      change_in_time <- as.numeric(difftime(current_timestamp, previous_timestamp), units = "secs")
      single_data$jer_linear[row] <- (current_acceleration[1] - previous_acceleration[1]) / change_in_time
    }
    previous_timestamp <- current_timestamp
    previous_acceleration <- current_acceleration 
  }
  rm(previous_timestamp)
  rm(previous_acceleration)
  rm(current_acceleration)
  rm(current_timestamp)
  single_data$jer_linear[is.na(single_data$jer_linear)] <- 0
  single_data$jer_linear_loess <- loess(single_data$jer_linear ~ ind, span=loess_smoothing_jer)$fitted
  
  ## Acceleration zero crossings
  single_data$acc_zero_crossing <- FALSE
  for(row in 2:nrow(single_data))
  {
    single_data$acc_zero_crossing[row] <-
      ((single_data$acc_linear_loess[row] * single_data$acc_linear_loess[row - 1]) < 0) & # Sign changed between this and previous acceleration
      abs(single_data$acc_linear_loess[row] - single_data$acc_linear_loess[row - 1]) >= acc_crossing_noise_threshold # Magnitude difference is greater than threshold (avoids noise)
  }
  
  ## Classify the movement sections
  single_data$movement_classification <- "None"
  for(row in 1:nrow(single_data))
  {
    if(single_data$acc_linear_loess[row] > movement_classification_acceleration_threshold)
    {
      single_data$movement_classification[row] <- "Accelerating"
    }
    else if(single_data$acc_linear_loess[row] < -movement_classification_acceleration_threshold)
    {
      single_data$movement_classification[row] <- "Slowing"
    }
  }
  
  single_data$movement_number <- -1
  movement_counter <- 0
  movement_in_progress <- FALSE
  previous_classification <- "None"
  for(row in 1:nrow(single_data))
  {
    if(single_data$movement_classification[row] == "Accelerating")
    {
      if(previous_classification == "Accelerating")
      {
        # Movement continuing
        single_data$movement_number[row] <- movement_counter
        movement_in_progress <- TRUE
      }
      else if(previous_classification == "Slowing")
      {
        # Movement started
        movement_counter <- movement_counter + 1
        single_data$movement_number[row] <- movement_counter
        movement_in_progress <- TRUE      
      }
      else if(previous_classification == "None")
      {
        # Movement started
        movement_counter <- movement_counter + 1
        single_data$movement_number[row] <- movement_counter
        movement_in_progress <- TRUE     
      }
    }
    else if(single_data$movement_classification[row] == "Slowing")
    {
      if(previous_classification == "Accelerating")
      {
        # Movement on down turn
        single_data$movement_number[row] <- movement_counter
      }
      else if(previous_classification == "Slowing")
      {
        # Slow continuing
        if(movement_in_progress)
        {
          single_data$movement_number[row] <- movement_counter
        }
      }
      else if(previous_classification == "None")
      {
        # Loess distortion?
      }    
    }
    else if(single_data$movement_classification[row] == "None")
    {
      if(previous_classification == "Accelerating")
      {
        # Movement ended (small)
        movement_in_progress <- FALSE  
      }
      else if(previous_classification == "Slowing")
      {
        # Movement ended
        movement_in_progress <- FALSE  
      }
      else if(previous_classification == "None")
      {
        # Robot is at standstill
        movement_in_progress <- FALSE 
      }    
    }
    previous_classification <- single_data$movement_classification[row]
  }
  rm(movement_counter)
  rm(movement_in_progress)
  rm(previous_classification)
  
  return(single_data)
}

get_movement_blocks <- function(data) {
  movement_blocks <- subset(data[match(unique(data$movement_number), data$movement_number), ], movement_number != -1) # Get starting frame of each movement
  movement_blocks <- select(movement_blocks, index, timestamp_r, movement_number)
  movement_blocks$timestamp_r_end <- as.POSIXct(NA)
  
  #Average values of each movement
  movement_blocks$average_vel <- -1
  movement_blocks$average_acc <- -1
  movement_blocks$average_jer <- -1
  for(block in 1:nrow(movement_blocks))
  {
    total_vel <- 0
    total_acc <- 0
    total_jer <- 0
    n <- 0
    movement_found <- FALSE
    for(point in data$index[block]:nrow(data))
    {
      if(data$movement_number[point] == movement_blocks$movement_number[block])
      {
        total_vel <- total_vel + abs(data$vel_linear_loess[point])
        total_acc <- total_acc + abs(data$acc_linear_loess[point])
        total_jer <- total_jer + abs(data$jer_linear_loess[point])
        n <- n + 1
        movement_found <- TRUE
      }
      else if(movement_found == TRUE)
      {
        movement_blocks$timestamp_r_end[block] <- data$timestamp_r[point]
        break
      }
    }
    if(n > 0)
    {
      movement_blocks$average_vel[block] <- (total_vel / n)
      movement_blocks$average_acc[block] <- (total_acc / n)
      movement_blocks$average_jer[block] <- (total_jer / n)
    }
  }
  
  movement_blocks$Participant.ID <- participant_id
  return(movement_blocks)
}

get_garmin_data <- function(participant_id, first_lap_start_timestamp, last_lap_end_timestamp) {
  garmin_data <- NA
  ## Locate file in subject's directory
  subject_dir <- file.path("..", "Subjects", participant_id)
  for(folder in (list.dirs(subject_dir, full.names=TRUE, recursive=FALSE)))
  {
    if(grepl("Garmin", folder, fixed = TRUE)) # Is the garmin watch folder in this subject's data
    {
      #print(participant_id)
      #print(folder)
      for(file in (list.files(folder, full.names=TRUE, recursive=FALSE)))
      {
        if(grepl("CONV.csv", file, fixed = TRUE)) # This is our converted, processed files
        {
          #print(file)
          garmin_data <- read.csv(file, header=TRUE, sep=",", na.strings = c("", "NA"))
          garmin_data$Participant.ID <- participant_id
          #garmin_data$timestamp_r <- strptime(garmin_data$timestamp, format="%Y-%m-%dT%H:%M:%OS", tz="America/New_York")
          
          #garmin_data$timestamp_r <- strptime(garmin_data$timestamp, format="%Y-%m-%dT%H:%M:%OS", tz="UTC")
          
          garmin_data$timestamp_r <- as.POSIXct(garmin_data$timestamp, format="%Y-%m-%dT%H:%M:%OS", tz="UTC", usetz=TRUE)
          garmin_data$timestamp_r <- format(garmin_data$timestamp_r, tz="America/New_York", usetz=TRUE) # Adjust timezone
          #garmin_data$timestamp_r2 <- garmin_data$timestamp_r - 
          
          break
        }
      }
      break
    }
  }
  ## Add the deviations from resting heart rate
  garmin_data$resting_heart_rate <- NA
  garmin_data$heart_rate_deviation <- NA
  if(class(garmin_data) == "data.frame")
  {
    ## Get non-action time heart rates
    #non_action_data <- subset(garmin_data, timestamp_r < first_lap_start_timestamp | timestamp_r > last_lap_end_timestamp)
    non_action_data <- subset(garmin_data, timestamp_r < first_lap_start_timestamp)
    resting_heart_rate <- round(mean(non_action_data$heart_rate, na.rm=TRUE))
    garmin_data$resting_heart_rate <- resting_heart_rate
    garmin_data$heart_rate_deviation <- garmin_data$heart_rate - resting_heart_rate
  }
  
  return(garmin_data)
}

get_empatica_data <- function(participant_id) {
  empatica_data <- NA
  
  ## Locate file in subject's directory
  subject_dir <- file.path("..", "Subjects", participant_id)
  acc <- NA
  bvp <- NA
  eda <- NA
  hr <- NA
  #ibi <- NA
  temp <- NA
  for(folder in (list.dirs(subject_dir, full.names=TRUE, recursive=FALSE)))
  {
    if(grepl("Empatica", folder, fixed = TRUE))
    {
      if(file.exists(file.path(folder, "ACC.csv")))
      {
        acc <- read.csv(file.path(folder, "ACC.csv"), header=TRUE, sep=",", na.strings = c("", "NA"))
      }
      if(file.exists(file.path(folder, "BVP.csv")))
      {
        bvp <- read.csv(file.path(folder, "BVP.csv"), header=TRUE, sep=",", na.strings = c("", "NA"))
      }
      if(file.exists(file.path(folder, "EDA.csv")))
      {
        eda <- read.csv(file.path(folder, "EDA.csv"), header=TRUE, sep=",", na.strings = c("", "NA"))
      }
      if(file.exists(file.path(folder, "HR.csv")))
      {
        hr <- read.csv(file.path(folder, "HR.csv"), header=TRUE, sep=",", na.strings = c("", "NA"))
      }
      # if(file.exists(file.path(folder, "IBI.csv")))
      # {
      #   ibi <- read.csv(file.path(folder, "IBI.csv"), header=TRUE, sep=",", na.strings = c("", "NA"))
      # }
      if(file.exists(file.path(folder, "TEMP.csv")))
      {
        temp <- read.csv(file.path(folder, "TEMP.csv"), header=TRUE, sep=",", na.strings = c("", "NA"))
      }
      # for(file in (list.files(folder, full.names=TRUE, recursive=FALSE)))
      # {
      #   if(grepl("HR.csv", file, fixed = TRUE)) # This is our converted, processed files
      #   {
      #     #print(file)
      #     empatica_data <- read.csv(file, header=TRUE, sep=",", na.strings = c("", "NA"))
      #     empatica_data$Participant.ID <- participant_id
      #     empatica_data$timestamp_r <- as.POSIXct(empatica_data$time_series, origin="1970-01-01", tz="America/New_York")
      #     #empatica_data$timestamp_r <- strptime(garmin_data$timestamp, format="%Y-%m-%dT%H:%M:%OS")
      #     break
      #   }
      # }
      break
    }
  }
  
  ## Combine
  #empatica_data <- eda
  #empatica_data <- hr
  #empatica_data <- ibi
  #ifelse(class(empatica_data) == "data.frame", empatica_data <- merge(empatica_data, bvp, by="time_series"), empatica_data <- bvp)
  
  ## Add timestamp in POSIXct
  if(class(hr) == "data.frame") # Any of the required files were found so data is valid
  {
    hr$timestamp_r <- as.POSIXct(hr$time_series, origin="1970-01-01", tz="America/New_York", usetz=TRUE)
  }
  if(class(eda) == "data.frame") # Any of the required files were found so data is valid
  {
    eda$timestamp_r <- as.POSIXct(eda$time_series, origin="1970-01-01", tz="America/New_York", usetz=TRUE)
  }
  if(class(temp) == "data.frame") # Any of the required files were found so data is valid
  {
    temp$timestamp_r <- as.POSIXct(temp$time_series, origin="1970-01-01", tz="America/New_York", usetz=TRUE)
  }
  if(class(bvp) == "data.frame") # Any of the required files were found so data is valid
  {
    bvp$timestamp_r <- as.POSIXct(bvp$time_series, origin="1970-01-01", tz="America/New_York", usetz=TRUE)
  }
  
  return(list(hr, eda, temp, bvp))
}

get_progress_tracker_data <- function(participant_id, single_data) {
  subject_dir <- file.path("..", "Subjects", participant_id, "ProgressTracker")
  pt_files <- sort(list.files(subject_dir, full.names=TRUE, recursive=FALSE), decreasing = TRUE)
  progress_tracker_data <- read.csv(pt_files[1], # Grab the first item on the sorted list (last saved item, can be manual or auto save)
                                    header=TRUE,
                                    skip=1,
                                    sep=",",
                                    na.strings = c("", "NA", "-1"))
  
  
  
  ## Convert timestamps to R-friendly
  progress_tracker_data$End_Timestamp_r <- as.POSIXct(progress_tracker_data$End_Timestamp, format="%m_%d_%Y-%H_%M_%OS", tz="America/New_York", usetz=TRUE)
  progress_tracker_data$Start_Timestamp_r <- as.POSIXct(progress_tracker_data$Start_Timestamp, format="%m_%d_%Y-%H_%M_%OS", tz="America/New_York", usetz=TRUE)
  
  ## Compute Zero-crossing data in movement_data to indicate "jerkiness" of movement.
  progress_tracker_data$Zero_Crossings <- NA
  for(i in 1:nrow(progress_tracker_data))
  {
    lap_movements <- subset(single_data, timestamp_r >= progress_tracker_data$Start_Timestamp_r[i] & timestamp_r < progress_tracker_data$End_Timestamp_r[i])
    if(nrow(lap_movements) == 0) # Not timestamps were found in this range. Likely that the robot offset is not set correctly. Double check manual participant log data.
    {
      print(paste0("No valid movement data found for participant ", participant_id, " for lap ", i))
      next  
    }
    
    #lap_movements <- filter(lap_movements, vel_linear_loess>=0.001) # Experimented with, didn't change anything
    
    zero_crossings <- 0
    prev_val <- lap_movements$jer_linear_loess[1]
    for(j in 2:nrow(lap_movements))
    {
      curr_val <- lap_movements$jer_linear_loess[j]
      if(!is.na(prev_val) & !is.na(curr_val))
      {
        if(curr_val * prev_val < 0) # Signs on values are different
        {
          zero_crossings <- zero_crossings + 1
        }       
      }
      prev_val <- curr_val
    }
    progress_tracker_data$Zero_Crossings[i] <- zero_crossings
  }
  progress_tracker_data$Zero_Crossings_Per_Second <- progress_tracker_data$Zero_Crossings / progress_tracker_data$Time
  
  return(progress_tracker_data)
}

add_lap_data_to_biometric <- function(participant_id, biometric_data, progress_tracker_data) {
  print(paste0("Adding lap information to biometric data for participant ", participant_id))
  if(class(biometric_data) != "data.frame" | class(progress_tracker_data) != "data.frame")
  {
    return(biometric_data)
  }
  
  biometric_data <- subset(biometric_data, Participant.ID == participant_id)
  #progress_tracker_data <- subset(progress_tracker_data, Participant.ID == participant_id)
  
  biometric_data$lap <- NA
  biometric_data$route <- NA
  for(i in 1:nrow(biometric_data))
  {
    bio_timestamp <- biometric_data$timestamp_r[i]
    lap <- NA
    route <- NA
    ## Find which lap this occurs in 
    for(j in 1:nrow(progress_tracker_data))
    {
      #print(participant_id)
      #print(progress_tracker_data$Start_Timestamp_r[j])
      #print(progress_tracker_data$End_Timestamp_r[j])
      if(is.na(progress_tracker_data$Start_Timestamp_r[j]) | is.na(progress_tracker_data$End_Timestamp_r[j])) # Missing lap info
      {
        next
      }
      if(bio_timestamp >= progress_tracker_data$Start_Timestamp_r[j] & bio_timestamp <= progress_tracker_data$End_Timestamp_r[j])
      {
        lap <- progress_tracker_data$Lap[j]
        route <- progress_tracker_data$Route[j]
        break
      }
    }
    biometric_data$lap[i] <- lap
    biometric_data$route[i] <- route
  }

  return(biometric_data) 
}
  
# log_data <- get_participant_log_data()
# single_data <- get_robot_movement_data(participant_id, get_participant_robot_time_offset(participant_id, log_data), acc_crossing_noise_threshold, movement_classification_acceleration_threshold)
# movement_blocks <- get_movement_blocks(single_data)
# progress_tracker_data <- get_progress_tracker_data(participant_id, single_data)
# garmin_data <- get_garmin_data(participant_id, min(progress_tracker_data$Start_Timestamp_r, na.rm=TRUE), max(progress_tracker_data$End_Timestamp_r, na.rm=TRUE))
# empatica_data <- get_empatica_data(participant_id)
# garmin_data <- add_lap_data_to_biometric(participant_id, garmin_data, progress_tracker_data)
