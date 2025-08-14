library(dplyr)
library(ggplot2)
library(reshape2)
library(DescTools) # ANOVA effect size
library(rstatix) # Cohen's D - t-test effect size
library(pastecs) # For general statistics on data set
library(vegan) # For non parametric MANOVA

output_dir <- "ROutput"

ar_color <- "#49D5F5"
tablet_color <- "#CAF781"
col3 <- "#F5B378"
col4 <- "#EABAF7"
col5 <- "#6595A0" 
col6 <- "#6C755E"

exclude_subjects <- c("4", "10") # Subjects to exclude (noted in Log)

## Load in scripts
source("single_robot.R")

###### Qualtrics ###### 
#qualtrics <- read.csv("../Subjects/FAKE_Qualtrics.csv",
qualtrics <- read.csv("../Subjects/Qualtrics.csv",
                                  #skip=1,
                                  header=TRUE,
                                  #sep=",")[-1,]
                                  sep=",",
                                  na.strings = c("", "NA"))
qualtrics <- qualtrics[qualtrics$Finished == "True", ] # Drop unfinished

qualtrics$Control <- as.factor(qualtrics$Control)
qualtrics$Complexity <- as.factor(qualtrics$Complexity)

###### SUS/TLX ######

## Score SUS
sus <- subset(qualtrics, select=grepl("SUS", colnames(qualtrics)) | colnames(qualtrics)=="Participant.ID")
sus$SUS.Score <- (
  (as.numeric(sus$SUS.1) - 1) +
    (5 - as.numeric(sus$SUS.2)) +
    (as.numeric(sus$SUS.3) - 1)) +
    (5 - as.numeric(sus$SUS.4)) +
    (as.numeric(sus$SUS.5) - 1) +
    (5 - as.numeric(sus$SUS.6)) +
    (as.numeric(sus$SUS.7) - 1) +
    (5 - as.numeric(sus$SUS.8)) +
    (as.numeric(sus$SUS.9) - 1) +
    (5 - as.numeric(sus$SUS.10)) * 2.5

## Score TLX
tlx <- subset(qualtrics, select=grepl("TLX", colnames(qualtrics)) | colnames(qualtrics)=="Participant.ID")
tlx$TLX.Mental_Demand <- (as.numeric(tlx$TLX.Mental_1) - 1) * 5
tlx$TLX.Physical_Demand <- (as.numeric(tlx$TLX.Physical_1) - 1) * 5
tlx$TLX.Temporal_Demand <- (as.numeric(tlx$TLX.Temporal_1) - 1) * 5
tlx$TLX.Performance <- (as.numeric(tlx$TLX.Performance_1) - 1) * 5
tlx$TLX.Effort <- (as.numeric(tlx$TLX.Effort_1) - 1) * 5
tlx$TLX.Frustration <- (as.numeric(tlx$TLX.Frustration_1) - 1) * 5

## Normalize TLX to 0 (poor) to 100 (ideal) scale - TLX.Performance was already this way
tlx$TLX.Mental_Demand <- abs(tlx$TLX.Mental_Demand - 100)
tlx$TLX.Physical_Demand <- abs(tlx$TLX.Physical_Demand - 100)
tlx$TLX.Temporal_Demand <- abs(tlx$TLX.Temporal_Demand - 100)
tlx$TLX.Effort <- abs(tlx$TLX.Effort - 100)
tlx$TLX.Frustration <- abs(tlx$TLX.Frustration - 100)

## Merge both back into dataset
#qualtrics <- merge(x = qualtrics, y = sus[c("Participant.ID", "SUS.Score")], by = "Participant.ID", all.x=TRUE)
#qualtrics <- merge(x = qualtrics, y = tlx[c("Participant.ID", "TLX.Mental_Demand", "TLX.Physical_Demand", "TLX.Temporal_Demand", "TLX.Performance", "TLX.Effort", "TLX.Frustration")], by = "Participant.ID", all.x=TRUE)

###### Trust ######

import_trust_measure <- function(data, column_identifier){
  trust_data <- subset(data, select=grepl(column_identifier, colnames(data)) | colnames(data)=="Participant.ID")
  trust_data <- na.omit(trust_data)
  for(i in 2:ncol(trust_data))
  {
    trust_data[, i] <- gsub('%', '', trust_data[, i]) 
    trust_data[, i] <- as.numeric(trust_data[, i])
  }
  return(trust_data)
}
trust_full <- import_trust_measure(qualtrics, "TRPSH.Full")
trust_short1 <- import_trust_measure(qualtrics, "TRPHS.Short1")
trust_short2 <- import_trust_measure(qualtrics, "TRPHS.Short2")

## Score
short_reverse_index <- c(3, 7, 8) + 1 # Account for participant ID column
full_reverse_index <- c(4, 9, 20, 22, 24) + 1 # Account for participant ID column
score_trust_measure <- function(data, length, reverse_index)
{
  data$PositiveValues <- rowSums(subset(data, select = -append(reverse_index, 1))) # Add up all positive, non-participant ID values
  data$NegativeValues <- rowSums(subset(data, select = reverse_index))
  data$Total <- data$PositiveValues - data$NegativeValues
  data$Score <- data$Total / length
  return(data$Score)
}
trust_full$Trust.Full.Score <- score_trust_measure(trust_full, ncol(trust_full) - 1, full_reverse_index)
trust_short1$Trust.Short1.Score <- score_trust_measure(trust_short1, ncol(trust_short1) - 1, short_reverse_index)
trust_short2$Trust.Short2.Score <- score_trust_measure(trust_short2, ncol(trust_short2) - 1, short_reverse_index)

## Merge back into data set
#qualtrics <- merge(x = qualtrics, y = trust_full[c("Participant.ID", "Trust.Full.Score")], by = "Participant.ID", all.x=TRUE)
#qualtrics <- merge(x = qualtrics, y = trust_short1[c("Participant.ID", "Trust.Short1.Score")], by = "Participant.ID", all.x=TRUE)
#qualtrics <- merge(x = qualtrics, y = trust_short2[c("Participant.ID", "Trust.Short2.Score")], by = "Participant.ID", all.x=TRUE)

##### Participant Log #####

log_data <- get_participant_log_data()

##### Robot Movement Data #####

# ## Consolidate all movement blocks from each participant's robot movement file
# movement_blocks <- NA
# for(subject_dir in (list.dirs(file.path("..", "Subjects"), full.names=TRUE, recursive=FALSE)))
# {
#   #print(subject_dir)
#   for(subject_file in (list.files(subject_dir, full.names=FALSE)))
#   {
#     #print(subject_file)
#     if(grepl("[0-9][0-9]-[0-9][0-9]-[0-9][0-9][0-9][0-9]_[0-9][0-9]-[0-9][0-9]-[0-9][0-9]-[0-9][0-9][0-9].csv", 
#             subject_file))
#     {
#       #print(subject_file)
#       participant_id <- strsplit(subject_dir, split = .Platform$file.sep)[[1]]
#       participant_id <- participant_id[length(participant_id)] # Get last thing in file path
#       single_data <- get_robot_movement_data(participant_id, subject_file, acc_crossing_noise_threshold, movement_classification_acceleration_threshold)
#       if(class(movement_blocks) != "data.frame")
#       {
#         movement_blocks <- get_movement_blocks(single_data)
#       }
#       else
#       {
#         movement_blocks <- rbind(movement_blocks, get_movement_blocks(single_data)) 
#       }
#     }
#   }
# }
# rm(participant_id)
# rm(single_data)

##### Garmin Watch Data #####
# 
# # for(subject_dir in (list.dirs(file.path("..", "Subjects"), full.names=TRUE, recursive=FALSE)))
# # {
# #   participant_id <- strsplit(subject_dir, split = .Platform$file.sep)[[1]]
# #   participant_id <- participant_id[length(participant_id)] # Get last thing in file path
# #   
# #   for(folder in (list.dirs(subject_dir, full.names=TRUE, recursive=FALSE)))
# #   {
# #     if(grepl("Garmin", folder, fixed = TRUE)) # Is the garmin watch folder in this subject's data
# #     {
# #       print(participant_id)
# #       #print(folder)
# #       for(file in (list.files(folder, full.names=TRUE, recursive=FALSE)))
# #       {
# #         if(grepl("CONV.csv", file, fixed = TRUE)) # This is our converted, processed files
# #         {
# #           print(file)
# #           qualtrics <- read.csv(file, header=TRUE, sep=",", na.strings = c("", "NA"))
# #         }
# #       }
# #     }
# #   }  
# # }
# 
# garmin_data <- NA
# for(subject_dir in (list.dirs(file.path("..", "Subjects"), full.names=TRUE, recursive=FALSE)))
# {
#   participant_id <- strsplit(subject_dir, split = .Platform$file.sep)[[1]]
#   participant_id <- participant_id[length(participant_id)] # Get last thing in file path
# 
#   single_garmin <- get_garmin_data(participant_id)
#   if(class(single_garmin) != "data.frame") # No Garmin data was found for this subject
#   {
#     next
#   }
#   else
#   {
#     if(class(garmin_data) != "data.frame")
#     {
#       garmin_data <- single_garmin
#     }
#     else
#     {
#       garmin_data <- rbind(garmin_data, single_garmin) 
#     }
#   }
# }
# rm(participant_id)
# rm(single_garmin)

##### Progress Tracker #####

# progress_tracker <- read.csv("../Subjects/FAKE_ConsolidatedProgressTrackers.csv",
#                              #skip=1,
#                              header=TRUE,
#                              #sep=",")[-1,]
#                              sep=",",
#                              na.strings = c("", "NA"))
# progress_tracker <- subset(progress_tracker, select = -c(X)) # Drop row name column (remnant from Python pandas)
# progress_tracker_2 <- progress_tracker[!(progress_tracker$Lap %in% c(1,4,7)), ] # Data set with the first lap in each set removed

## All
progress_tracker <- NA
for(subject_dir in (list.dirs(file.path("..", "Subjects"), full.names=TRUE, recursive=FALSE)))
{
  participant_id <- strsplit(subject_dir, split = .Platform$file.sep)[[1]]
  participant_id <- participant_id[length(participant_id)] # Get last thing in file path
  
  if(participant_id %in% exclude_subjects)
  {
    next
  }
  
  single_pt <- get_progress_tracker_data(participant_id, get_robot_movement_data(participant_id,
                                                                                 get_participant_robot_time_offset(participant_id, log_data),
                                                                                 acc_crossing_noise_threshold,
                                                                                 movement_classification_acceleration_threshold))
  if(class(single_pt) != "data.frame") # No data was found for this subject
  {
    next
  }
  else
  {
    if(class(progress_tracker) != "data.frame")
    {
      progress_tracker <- single_pt
    }
    else
    {
      progress_tracker <- rbind(progress_tracker, single_pt) 
    }
  }
}
rm(participant_id)
rm(single_pt)

## First lap removed
progress_tracker_first_removed <- progress_tracker[!(progress_tracker$Lap %in% c(1,4,7)), ] # Data set with the first lap in each set removed

## Best from each lap
progress_tracker_best <- progress_tracker
for(participant in unique(progress_tracker$Participant.ID))
{
  for(route in unique(progress_tracker$Route))
  {
    # Find best lap in for each route
    valid_laps <- subset(progress_tracker, Participant.ID==participant & Route==route)
    valid_laps <- na.omit(valid_laps) # Account for missing laps for some reason (check Log for notes)
    best_time <- min(valid_laps$Time)
    # Remove laps that aren't this from the data
    progress_tracker_best <- progress_tracker_best[-which(progress_tracker_best$Participant.ID==participant & progress_tracker_best$Route==route & progress_tracker_best$Time!=best_time,),]
  }
}
progress_tracker_best <- na.omit(progress_tracker_best) # Account for missing laps for some reason (check Log for notes)
rm(best_time)
rm(valid_laps)

##### Consolidate Logs ##### 
data <- qualtrics
data <- merge(x = data, y = trust_full[c("Participant.ID", "Trust.Full.Score")], by = "Participant.ID", all.x=TRUE)
data <- merge(x = data, y = trust_short1[c("Participant.ID", "Trust.Short1.Score")], by = "Participant.ID", all.x=TRUE)
data <- merge(x = data, y = trust_short2[c("Participant.ID", "Trust.Short2.Score")], by = "Participant.ID", all.x=TRUE)
data <- merge(x = data, y = sus[c("Participant.ID", "SUS.Score")], by = "Participant.ID", all.x=TRUE)
data <- merge(x = data, y = tlx[c("Participant.ID", "TLX.Mental_Demand", "TLX.Physical_Demand", "TLX.Temporal_Demand", "TLX.Performance", "TLX.Effort", "TLX.Frustration")], by = "Participant.ID", all.x=TRUE)

##### Watch Data #####

garmin_data <- NA
for(subject_dir in (list.dirs(file.path("..", "Subjects"), full.names=TRUE, recursive=FALSE)))
{
  participant_id <- strsplit(subject_dir, split = .Platform$file.sep)[[1]]
  participant_id <- participant_id[length(participant_id)] # Get last thing in file path
  
  if(participant_id %in% exclude_subjects)
  {
    next
  }
  
  single_pt <- subset(progress_tracker, Participant.ID == participant_id)
  
  single_garmin <- get_garmin_data(participant_id, min(single_pt$Start_Timestamp_r, na.rm=TRUE), max(single_pt$End_Timestamp_r, na.rm=TRUE))
  single_garmin <- add_lap_data_to_biometric(participant_id, single_garmin, single_pt)
  
  if(class(single_garmin) != "data.frame") # No data was found for this subject
  {
    next
  }
  else
  {
    if(class(garmin_data) != "data.frame")
    {
      garmin_data <- single_garmin
    }
    else
    {
      garmin_data <- rbind(garmin_data, single_garmin) 
    }
  }
}
rm(participant_id)
rm(single_pt)
rm(single_garmin)

## Mean from each lap
garmin_grouped <- garmin_data %>%
  group_by(Participant.ID, lap, route) %>%
  summarize(heart_rate_deviation_mean = mean(heart_rate_deviation))

## Add condition variables to grouped data frame
garmin_grouped$Control <- NA
garmin_grouped$Complexity <- NA
for(i in 1:nrow(garmin_grouped))
{
  control <- NA
  complexity <- NA
  for(j in 1:nrow(data))
  {
    if(garmin_grouped$Participant.ID[i] == data$Participant.ID[j])
    {
      #print(data$Control[j])
      control <- data$Control[j]
      complexity <- data$Complexity[j]
      break
    }
  }
  garmin_grouped$Control[i] <- control
  garmin_grouped$Complexity[i] <- complexity
}
garmin_grouped <- garmin_grouped[!is.na(garmin_grouped$lap),]
garmin_grouped$Control <- as.factor(garmin_grouped$Control)
garmin_grouped$Complexity <- as.factor(garmin_grouped$Complexity)
rm(control)
rm(complexity)

## Clean environment
rm(trust_full)
rm(trust_short1)
rm(trust_short2)
rm(sus)
rm(tlx)

##### Progress Tracker ANOVAs #####

sink(file.path(output_dir, "progress_tracker.txt"))

## Numbers
progress_tracker %>%
  group_by(Control, Complexity, Route) %>%
  summarize(
    count = n(),
    mean = mean(Time),
    sd = sd(Time)
  )

## Check assumption of normality
#aggregate(cbind(P.value=Time) ~ Control + Complexity + Route, progress_tracker, FUN = function(x) shapiro.test(x)$p.value)
#shapiro.test(subset(progress_tracker, Control=="AR" & Complexity=="Outside" & Route=="Forward")$Time)

## ANOVA (Route included)
res <- aov(Time ~ Control * Route, data = progress_tracker_best)
print(summary(res))
#print(TukeyHSD(res, which = "Route"))
pairwise_t_test(
  Time ~ Route,
  data=progress_tracker_best,
  paired = TRUE,
  alternative = "two.sided",
  p.adjust.method = "BH"
)
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
print(EtaSq(res, type=2, anova=FALSE)) # Effect size
#plot(res, 2)

## ANOVAs (Routes seperated)
# Forward
res <- aov(Time ~ Control, data = subset(progress_tracker_best, Route=="Forward"))
print(summary(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
print(EtaSq(res, type=2, anova=FALSE)) # Effect size
# Backward
res <- aov(Time ~ Control, data = subset(progress_tracker_best, Route=="Backward"))
print(summary(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
print(EtaSq(res, type=2, anova=FALSE)) # Effect size
# Combined
res <- aov(Time ~ Control, data = subset(progress_tracker_best, Route=="Combined"))
print(summary(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
print(EtaSq(res, type=2, anova=FALSE)) # Effect size

## Two way Kruskal Wallis? (Non-parametric two way anova)
# TODO

## Friedman Test
#friedman.test(Time ~ Control | Complexity, data=progress_tracker)
#friedman_data <- melt(subset(progress_tracker, select = c("Control", "Complexity", "Route", "Time")),
#     id = c("Control", "Complexity", "Route"))

# Jerk Zero-Crossings
res <- aov(Zero_Crossings_Per_Second ~ Control, data = subset(progress_tracker))
print(summary(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
print(EtaSq(res, type=2, anova=FALSE)) # Effect size

res <- aov(Zero_Crossings_Per_Second ~ Control, data = subset(progress_tracker_first_removed))
print(summary(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
print(EtaSq(res, type=2, anova=FALSE)) # Effect size

res <- aov(Zero_Crossings_Per_Second ~ Control, data = subset(progress_tracker_best))
print(summary(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
print(EtaSq(res, type=2, anova=FALSE)) # Effect size

# Jerk T-test
shapiro.test(progress_tracker$Zero_Crossings_Per_Second)
var.test(progress_tracker$Zero_Crossings_Per_Second ~ Control, data = progress_tracker)
res <- t.test(Zero_Crossings_Per_Second ~ Control,
              data = progress_tracker,
              alternative = "two.sided",
              paired = FALSE,
              var.equal = TRUE,
              conf.level = 0.95)
print(res)
cohens_d(Zero_Crossings_Per_Second ~ Control,
         data = progress_tracker,
         var.equal = TRUE)
stat.desc(subset(progress_tracker, Control=="Tablet")$Zero_Crossings_Per_Second)
stat.desc(subset(progress_tracker, Control=="AR")$Zero_Crossings_Per_Second)

sink()

## Progress Tracker (First laps in each set removed)

sink(file.path(output_dir, "progress_tracker_best.txt"))

# progress_tracker[!(progress_tracker$Lap %in% c(1,4,7)), ] %>%
#   group_by(Control, Complexity, Route) %>%
#   summarize(
#     count = n(),
#     mean = mean(Time),
#     sd = sd(Time)
#   )

sink()

##### Trust ANOVA #####

data %>%
  group_by(Control, Complexity) %>%
  summarize(
    count = n(),
    mean_short1 = mean(Trust.Short1.Score),
    sd_short1 = sd(Trust.Short1.Score),
    mean_short2 = mean(Trust.Short2.Score),
    sd_short2 = sd(Trust.Short2.Score),
    mean_full = mean(Trust.Full.Score),
    sd_full = sd(Trust.Full.Score)
  )

## MANOVA (All sections of trust)
res <- manova(cbind(Trust.Short1.Score, Trust.Short2.Score, Trust.Full.Score) ~ Control, data = data)
print(summary(res))
print(summary.aov(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
#print(EtaSq(res, type=2, anova=FALSE)) # Does not work with MANOVA
#plot(res, 2)

## ANOVA (Final trust across different conditions - Should be the same as last component of MANOVA)
res <- aov(Trust.Full.Score ~ Control, data = data)
print(summary(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
print(EtaSq(res, type=2, anova=FALSE)) # Effect size

## Trust over time
trust_over_time <- melt(data[,c("Participant.ID","Control","Complexity","Trust.Full.Score","Trust.Short1.Score", "Trust.Short2.Score")],
                        id=c("Participant.ID", "Control", "Complexity"),
                        variable.name="Test",
                        value.name="Score")
# res <- aov(Score ~ Control * Complexity * Test + Error(Test/(Control * Complexity)), data = trust_over_time) # Specify the within-subjects factors using the Error() term
# print(summary(res))

trust_over_time$Participant.ID <- as.factor(trust_over_time$Participant.ID)
trust_over_time$Test <- as.factor(trust_over_time$Test)
res <- aov(Score ~ Control * Test + Error(Participant.ID/(Control)), data = trust_over_time)
print(summary(res))
#trust_over_time %>% group_by(Control) %>%
pairwise_t_test(
  Score ~ Test,
  data=trust_over_time,
  paired = FALSE,
  alternative = "two.sided",
  p.adjust.method = "BH"
)

## Final trust t-test
shapiro.test(data$Trust.Full.Score)
var.test(Trust.Full.Score ~ Control, data = data)
res <- t.test(Trust.Full.Score ~ Control,
              data = data,
              alternative = "two.sided",
              paired = FALSE,
              var.equal = TRUE,
              conf.level = 0.95)
print(res)
cohens_d(Trust.Full.Score ~ Control,
         data = data,
         var.equal = TRUE)
stat.desc(subset(data, Control=="Tablet")$Trust.Full.Score)
stat.desc(subset(data, Control=="AR")$Trust.Full.Score)

##### Usability ANOVA #####

data %>%
  group_by(Control, Complexity) %>%
  summarize(
    count = n(),
    mean_SUS = mean(SUS.Score),
    sd_SUS = sd(SUS.Score),
    mean_TLX_Mental = mean(TLX.Mental_Demand),
    sd_TLX_Mental = sd(TLX.Mental_Demand),
    mean_TLX_Physical = mean(TLX.Physical_Demand),
    sd_TLX_Physical = sd(TLX.Physical_Demand),
    mean_TLX_Temportal = mean(TLX.Temporal_Demand),
    sd_TLX_Temportal = sd(TLX.Temporal_Demand),
    mean_TLX_Performance = mean(TLX.Performance),
    sd_TLX_Performance = sd(TLX.Performance),
    mean_TLX_Effort = mean(TLX.Effort),
    sd_TLX_Effort = sd(TLX.Effort),
    mean_TLX_Frustration = mean(TLX.Frustration),
    sd_TLX_Frustration = sd(TLX.Frustration)
  )

## MANOVA (All usability metrics)
res <- manova(cbind(SUS.Score,
                    TLX.Mental_Demand,
                    TLX.Physical_Demand,
                    TLX.Temporal_Demand,
                    TLX.Performance,
                    TLX.Effort,
                    TLX.Frustration) ~ Control, data = data)
print(summary(res))
print(summary.aov(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
#print(EtaSq(res, type=2, anova=FALSE)) # Effect size

## Multiple non-parametric t-tests (All usability metrics)
t_test_results <- lapply(c("SUS.Score", "TLX.Mental_Demand", "TLX.Physical_Demand", "TLX.Temporal_Demand", "TLX.Performance", "TLX.Effort", "TLX.Frustration"), function(var) {
  formula <- as.formula(paste(var, "~ Control"))
  #stat.desc(subset(data, Control=="XR")[var])
  #stat.desc(subset(data, Control=="Tablet")[var])
  t.test(formula,
         data = data,
         alternative = "two.sided",
         paired = FALSE,
         var.equal = FALSE,
         conf.level = 0.95)
  # wilcox.test(formula,
  #             data = data,
  #             alternative = "two.sided",
  #             paired = FALSE,
  #             exact = FALSE,
  #             conf.level = 0.95,
  #             conf.int = TRUE)
})
lapply(c("SUS.Score", "TLX.Mental_Demand", "TLX.Physical_Demand", "TLX.Temporal_Demand", "TLX.Performance", "TLX.Effort", "TLX.Frustration"), function(var) {
  stat.desc(subset(data, Control=="AR")[var])
})
lapply(c("SUS.Score", "TLX.Mental_Demand", "TLX.Physical_Demand", "TLX.Temporal_Demand", "TLX.Performance", "TLX.Effort", "TLX.Frustration"), function(var) {
  stat.desc(subset(data, Control=="Tablet")[var])
})
print(t_test_results)

lapply(c("SUS.Score", "TLX.Mental_Demand", "TLX.Physical_Demand", "TLX.Temporal_Demand", "TLX.Performance", "TLX.Effort", "TLX.Frustration"), function(var) {
  formula <- as.formula(paste(var, "~ Control"))
  cohens_d(formula,
           data = data,
           var.equal = FALSE)
})

sapply(t_test_results, function(result) result$data.name)
sapply(t_test_results, function(result) result$p.value)
p.adjust(sapply(t_test_results, function(result) result$p.value), method="BH")

##### Biometrics ANOVAs #####

res <- aov(heart_rate_deviation_mean ~ Control, data = garmin_grouped)
print(summary(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
print(EtaSq(res, type=2, anova=FALSE)) # Effect size

## Is Lap 1 different?
res <- aov(heart_rate_deviation_mean ~ lap, data = garmin_grouped)
print(summary(res))
print(shapiro.test(x = residuals(object = res))) # Assumption of normality
print(EtaSq(res, type=2, anova=FALSE)) # Effect size

## Deviation t-test
shapiro.test(garmin_grouped$heart_rate_deviation_mean) # Was not normally distributed
var.test(garmin_grouped$heart_rate_deviation_mean ~ Control, data = garmin_grouped)
res <- t.test(heart_rate_deviation_mean ~ Control,
              data = garmin_grouped,
              alternative = "two.sided",
              paired = FALSE,
              var.equal = FALSE,
              conf.level = 0.95)
print(res)
cohens_d(heart_rate_deviation_mean ~ Control,
         data = garmin_grouped,
         var.equal = FALSE)
stat.desc(subset(garmin_grouped, Control=="2")$heart_rate_deviation_mean) # Tablet
stat.desc(subset(garmin_grouped, Control=="1")$heart_rate_deviation_mean) # AR


###### Plots ######

##### Trust Over Time #####

plot_data <- melt(subset(data, select = c("Control", "Complexity", "Trust.Short1.Score", "Trust.Short2.Score", "Trust.Full.Score")),
                  id = c("Control", "Complexity"))
plot_data <- na.omit(plot_data)
plot_data <- subset(plot_data, Complexity=="Inside")
#plot_data_sum <- plot_data %>%
#  group_by(Control, Complexity, variable) %>%
#  summarise(Mean=mean(value),
#            SD=sd(value))
plot <- ggplot(plot_data,
               aes(x=factor(plot_data$variable, levels = c("Trust.Short1.Score", "Trust.Short2.Score", "Trust.Full.Score")), 
                   y=value,
                   color=Control)) +
  geom_point() +
  # geom_boxplot(aes(fill=Control),
  #              alpha=0.7,
  #              width=0.6,
  #              size=1,
  #              color="black",
  #              position=position_dodge(width=0.7),
  #              outlier.shape=NA) +
  stat_summary(aes(x=variable, y=value, group=Control),
               geom = "line",
               fun.y = "mean",
               size = 2,
               show.legend = FALSE) +
  stat_summary(aes(x=variable, y=value, fill=Control),
               geom = "point",
               fun.y = "mean",
               col = "black",
               size = 4,
               stroke=1.2,
               shape = 21) +
  # facet_grid(.~Complexity,
  #            space="fixed",
  #            margins = "vs") +
  ylab("Trust Score") +
  scale_x_discrete(
    "",
    labels = c(
      "Trust.Short1.Score" = "After Forward Route",
      "Trust.Short2.Score" = "After Backward Route",
      "Trust.Full.Score" = "After Hybrid Route")
  ) +
  scale_fill_manual(values=c(AR=ar_color, Tablet=tablet_color)) +
  scale_color_manual(values=c(AR=ar_color, Tablet=tablet_color)) +
  ylim(0, 100) + 
  theme_bw()
print(plot)
rm(plot)
rm(plot_data)

##### Usability #####

plot_data <- melt(subset(data, select = c("Control", "Complexity", "SUS.Score", "TLX.Mental_Demand", "TLX.Physical_Demand", "TLX.Temporal_Demand", "TLX.Performance", "TLX.Effort", "TLX.Frustration")),
                  id = c("Control", "Complexity"))
plot_data <- subset(plot_data, Complexity=="Inside")
plot <- ggplot(plot_data, aes(x=factor(plot_data$variable, levels = c("SUS.Score", "TLX.Mental_Demand", "TLX.Physical_Demand", "TLX.Temporal_Demand", "TLX.Performance", "TLX.Effort", "TLX.Frustration")),
                              y=value,
                              fill=Control)) +
  # geom_violin(color="black",
  #             draw_quantiles=TRUE,
  #             trim=TRUE,
  #             alpha=0.7,
  #             #linewidth=0.8,
  #             linewidth=0.6,
  #             adjust=4.0,
  #             position=position_dodge(width=0.7)) +
  geom_boxplot(alpha=0.7,
               width=0.5,
               size=1,
               color="black",
               position=position_dodge(width=0.7),
               outlier.shape=NA) +
  # stat_summary(color="black",
  #              #aes(color=Condition),
  #              show.legend = FALSE,
  #              fun = "mean",
  #              geom = "point", 
  #              #width = 0.5,
  #              position=position_dodge(width=0.9)) +#,
  # facet_grid(.~Complexity,
  #            space="fixed",
  #            margins = "vs") +
  ylab("Score (0 = poor, 100 = ideal)") +
  scale_x_discrete(
    "Usability Metric",
    labels = c(
      "SUS.Score" = "SUS",
      "TLX.Mental_Demand" = "Mental\nDemand",
      "TLX.Physical_Demand" = "Physical\nDemand",
      "TLX.Temporal_Demand" = "Temporal\nDemand",
      "TLX.Performance" = "Performance",
      "TLX.Effort" = "Effort",
      "TLX.Frustration" = "Frustration"),
    limits=rev) +
  scale_y_continuous(limits = c(0, 100)) +
  scale_fill_manual(values=c(AR=ar_color, Tablet=tablet_color)) +
  coord_flip() +
  theme_bw()
print(plot)
rm(plot)
rm(plot_data)

##### Progress Tracker Time Distributions #####

plot_data <- subset(progress_tracker_best, Complexity=="Inside")
plot <- ggplot(plot_data, aes(x=factor(plot_data$Route, levels = c("Combined", "Backward", "Forward")),
                              y=Time,
                              fill=Control)) +
  geom_boxplot(alpha=0.7,
               width=0.5,
               size=1,
               color="black",
               position=position_dodge(width=0.7),
               outlier.shape=NA) +
  # geom_violin(color="black",
  #             draw_quantiles=TRUE,
  #             trim=TRUE,
  #             alpha=0.7,
  #             #linewidth=0.8,
  #             linewidth=1.0,
  #             adjust=4.0,
  #             position=position_dodge(width=0.7)) +
  # stat_summary(color="black",
  #              #aes(color=Condition),
  #              show.legend = FALSE,
  #              fun = "mean",
  #              geom = "point",
  #              #width = 0.5,
  #              position=position_dodge(width=0.7)) +#,
  # facet_grid(.~Complexity,
  #            space="fixed",
  #            margins = "vs") +
  ylab("Best Lap Time (seconds)") +
  scale_x_discrete(
    "",
    labels = c(
      "Forward" = "Forward\nRoute",
      "Backward" = "Backward\nRoute",
      "Combined" = "Hybrid\nRoute")
    ) +
  scale_fill_manual(values=c(AR=ar_color, Tablet=tablet_color)) +
  coord_flip() +
  theme_bw()
print(plot)
rm(plot)
rm(plot_data)

##### Zero-Crossings Per Second #####

plot_data <- subset(progress_tracker_best, Complexity=="Inside")
plot <- ggplot(plot_data, aes(x=factor(plot_data$Route, levels = c("Combined", "Backward", "Forward")),
                              y=Zero_Crossings_Per_Second,
                              fill=Control)) +
  geom_violin(color="black",
              draw_quantiles=TRUE,
              trim=TRUE,
              alpha=0.7,
              #linewidth=0.8,
              linewidth=1,
              adjust=4.0,
              position=position_dodge(width=0.7)) +
  stat_summary(color="black",
               #aes(color=Condition),
               show.legend = FALSE,
               fun = "mean",
               geom = "point", 
               #width = 0.5,
               position=position_dodge(width=0.7)) +#,
  # facet_grid(.~Complexity,
  #            space="fixed",
  #            margins = "vs") +
  scale_fill_manual(values=c(AR=ar_color, Tablet=tablet_color)) +
  ylab("Jerk Zero-Crossings Per Second") +
  scale_x_discrete(
    "",
    labels = c(
      "Forward" = "Forward\nRoute",
      "Backward" = "Backward\nRoute",
      "Combined" = "Hybrid\nRoute")
  ) +
  scale_y_continuous(limits = c(0.0, 2.0)) +
  coord_flip() +
  theme_bw()
print(plot)
rm(plot)
rm(plot_data)