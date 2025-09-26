import heapq

def assign_tasks(factor, arrival, bonus, reward, duration, time_bonus):
    T = len(arrival)
    P = len(factor[0])

    # Initialize the schedule
    schedule = [None] * T
    processor_available_time = [0] * P

    # Create a priority queue with tasks sorted by descending priority
    tasks = []
    for i in range(T):
        potential_reward = max(factor[i]) * (reward[i] + bonus[i]) / duration[i]
        heapq.heappush(tasks, (-potential_reward, i))  # Using heapq to create a maxheap

    while tasks:
        _, i = heapq.heappop(tasks)
        
        best_reward = float('-inf')
        best_processor = None
        best_start_time = None
        
        # Try to schedule the task on each processor
        for p in range(P):
            start_time = max(arrival[i], processor_available_time[p])
            
            # Calculate the reward with and without the bonus
            if start_time < arrival[i] + time_bonus[i]:
                reward_with_bonus = factor[i][p] * (
                    bonus[i] + reward[i] * duration[i] / (duration[i] + start_time - arrival[i])
                )
            else:
                reward_with_bonus = factor[i][p] * reward[i] * duration[i] / (duration[i] + start_time - arrival[i])
                
            # Check if this processor and start time give a better reward
            if reward_with_bonus > best_reward:
                best_reward = reward_with_bonus
                best_processor = p
                best_start_time = start_time

        # Schedule the task on the best processor found and update availability
        schedule[i] = (best_processor, best_start_time)
        processor_available_time[best_processor] = best_start_time + duration[i]

    return schedule
