#!/usr/bin/env python
import tensorflow as tf

"""
This file contains tensorflow operations to generate event timestamp images in tensorflow.
CURRENTLY NOT WORKING.
"""

def gen_event_timestamp_images(events, image_size):
    pol = events[3, :]

    pos_event_positions = tf.where(tf.equal(pol, 1))
    neg_event_positions = tf.where(tf.equal(pol, 0))

    pos_events = tf.gather(events,
                           pos_event_positions)
    neg_events = tf.gather(events,
                           neg_event_positions)
    pos_timestamp_image = gen_timestamp_image(pos_events, image_size)
    neg_timestamp_image = gen_timestamp_image(neg_events, image_size)
    pos_count_image = gen_count_image(pos_events, image_size)
    neg_count_image = gen_count_image(neg_events, image_size)

    event_timestamp_image = tf.stack([pos_count_image,
                                      neg_count_image,
                                      pos_timestamp_image,
                                      neg_timestamp_image],
                                     axis=0)
    
    return event_timestamp_image

def gen_count_image(events, image_size):
    n_events = events.shape[1]
    map_shape = [image_size[1], image_size[0]]
    
    count_image = tf.scatter_nd(tf.cast(tf.round(events[:2, :]), tf.int32),
                                tf.ones(n_events, 1),
                                map_shape)
    return count_image

def gen_timestamp_image(events, image_size):
    n_events = tf.shape(events)[1]
    #n_events = events.shape[1]
    ts = tf.squeeze(events[2, :])
    
    # Convert the xy positions to 
    inds = tf.round(events[1, :]) + (tf.round(events[0, :]) * image_size[0])
    
    # Sort events by pixel position so that neighboring pixels are next to each other.
    sorted_vals, sorted_inds = tf.nn.top_k(inds, n_events)
    # Reverse the outputs so it's in increasing order.
    sorted_vals = tf.reverse(sorted_vals, axis=[-1])
    sorted_inds = tf.reverse(sorted_inds, axis=[-1])
        
    map_shape = [image_size[1], image_size[0]]
    # Grab the timestamps corresponding to the sorted positions.
    t_sorted = tf.gather(ts, sorted_inds)
    # Find the unique pixel positions.
    (unique_inds, _, count) = tf.unique_with_counts(sorted_vals)
    # Convert the unique_inds back to x,y positions.
    unique_y = tf.mod(unique_inds, image_size[0])
    unique_x = tf.floordiv(unique_inds, image_size[0])
    unique_xy = tf.stack([unique_x, unique_y], axis=1)
    
    # The cumsum finds the position of the last appearance of each pixel. Grab their times.
    count_cumsum = tf.cumsum(count) - 1
    unique_ts = tf.gather(t_sorted, count_cumsum)

    print t_sorted.shape
    print unique_xy.shape
    print unique_ts.shape
    
    # Scatter them into an image.
    time_image = tf.scatter_nd(tf.cast(unique_xy, tf.int32),
                               unique_ts,
                               map_shape)
    return time_image
