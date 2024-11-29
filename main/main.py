
import kivy
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.textinput import TextInput
from kivy.uix.button import Button
from kivy.clock import Clock
import requests
from kivy_garden.graph import Graph, LinePlot
import logging

logging.getLogger("urllib3").setLevel(logging.WARNING)
logging.getLogger("kivy").setLevel(logging.ERROR)

# Rest of your code...


class KADAM(App):
    def build(self):
        layout = BoxLayout(orientation='vertical')
        
        # Create a graph widget
        self.graph = Graph(
            xlabel='Time',
            ylabel='Value',
            x_ticks_minor=5,
            x_ticks_major=1,
            y_ticks_major=10,
            y_grid_label=True,
            x_grid_label=True,
            padding=5,
            x_grid=True,
            y_grid=True,
            xmin=0,
            xmax=20,
            ymin=-99,  # Initial minimum y-axis value
            ymax=99    # Initial maximum y-axis value
        )
        self.graph.size_hint = (1, 0.6)
        # Create LinePlot instances for sensor and threshold data
        self.sensor_plot = LinePlot(line_width=1.5, color=[0, 1, 0, 1])  # Green for sensor data
        self.threshold_plot = LinePlot(line_width=1.5, color=[1, 0, 0, 1])  # Red for threshold
        
        # Add plots to the graph
        self.sensor_plot.points = []
        self.threshold_plot.points = []
        self.graph.add_plot(self.sensor_plot)
        self.graph.add_plot(self.threshold_plot)
        
        # Add the graph widget to the layout
        layout.add_widget(self.graph)
        
        # Integer input field with hint text
        self.message_input = TextInput(hint_text='Enter an integer', input_filter='int')
        self.message_input.size_hint = (1, 0.1)
        send_button = Button(text='Send Integer')
        send_button.size_hint = (1, 0.1)
        send_button.bind(on_press=self.send_message)
        
        layout.add_widget(self.message_input)
        layout.add_widget(send_button)
        
        # Initialize time counter
        self.time = 0
        
        # Schedule to update the plot every second
        Clock.schedule_interval(self.update_plot, 0.1)
        
        return layout

    def send_message(self, instance):
        try:
            # Convert input to an integer
            message = int(self.message_input.text)
            url = "http://192.168.4.1/message"  # Replace with ESP32 IP address
            
            # Send the integer as plain text
            headers = {'Content-Type': 'text/plain'}
            response = requests.post(url, data=str(message), headers=headers)
            
            print("Response from ESP32:", response.text)
        except ValueError:
            print("Please enter a valid integer.")
        except Exception as e:
            print("Error:", e)
    
    def update_plot(self, dt):
        try:
            # Request sensor and threshold data from ESP32
            url = "http://192.168.4.1/random"  # Endpoint for sensor and threshold data
            response = requests.get(url)
            
            if response.status_code == 200:
                try:
                    # Parse JSON response
                    data = response.json()
                    sensor_data = data.get("sensor", 0)
                    threshold_data = data.get("threshold", 0)
                except ValueError:  # Handle non-JSON response
                    print("Invalid JSON received:", response.text)
                    return
                
                # Print sensor data to the terminal
                print(f"Sensor Data: {sensor_data}")
                
                # Update y-axis label to display the threshold value
                self.graph.ylabel = f"Thres: {threshold_data}"
                
                # Dynamically adjust y-axis limits if needed
                min_y = min(sensor_data, threshold_data)
                max_y = max(sensor_data, threshold_data)
                if min_y < self.graph.ymin:
                    self.graph.ymin = min_y - 10  # Add margin
                if max_y > self.graph.ymax:
                    self.graph.ymax = max_y + 10  # Add margin
                
                # Update plots
                if len(self.sensor_plot.points) > 20:  # Keep the last 20 points
                    self.sensor_plot.points.pop(0)
                if len(self.threshold_plot.points) > 20:  # Keep the last 20 points
                    self.threshold_plot.points.pop(0)
                
                # Append new points
                self.sensor_plot.points.append((self.time, sensor_data))
                self.threshold_plot.points.append((self.time, threshold_data))
                
                self.time += 1  # Increment the time for the next point
                
                # Adjust x-axis to move with new points after 20 points
                if self.time > 20:
                    self.graph.xmin = self.time - 20
                    self.graph.xmax = self.time
    
            else:
                print("Failed to retrieve data from ESP32:", response.status_code)
        
        except requests.RequestException as e:
            print("Error updating plot:", e)




if __name__ == '__main__':
    KADAM().run()






# import kivy
# from kivy.app import App
# from kivy.uix.boxlayout import BoxLayout
# from kivy.uix.textinput import TextInput
# from kivy.uix.button import Button
# from kivy.clock import Clock
# import requests
# from kivy_garden.graph import Graph, LinePlot
# import logging

# logging.getLogger("urllib3").setLevel(logging.WARNING)
# logging.getLogger("kivy").setLevel(logging.ERROR)

# # Rest of your code...


# class KADAM(App):
#     def build(self):
#         layout = BoxLayout(orientation='vertical')
        
#         # Adjust graph size
#         self.graph = Graph(
#             xlabel='Time',
#             ylabel='Value',
#             x_ticks_minor=5,
#             x_ticks_major=1,
#             y_ticks_major=10,
#             y_grid_label=True,
#             x_grid_label=True,
#             padding=5,
#             x_grid=True,
#             y_grid=True,
#             xmin=0,
#             xmax=20,
#             ymin=-99,
#             ymax=99
#         )
#         self.graph.size_hint = (1, 0.7)  # Graph takes 70% of the height
#         layout.add_widget(self.graph)
        
#         # Adjust input field size
#         self.message_input = TextInput(hint_text='Enter an integer', input_filter='int')
#         self.message_input.size_hint = (1, 0.1)  # Input takes 10% of the height
        
#         # Adjust button size
#         send_button = Button(text='Send Integer')
#         send_button.size_hint = (1, 0.1)  # Button takes 10% of the height
#         send_button.bind(on_press=self.send_message)
        
#         layout.add_widget(self.message_input)
#         layout.add_widget(send_button)
        
#         self.time = 0
#         Clock.schedule_interval(self.update_plot, 0.1)
        
#         return layout

#     def send_message(self, instance):
#         try:
#             # Convert input to an integer
#             message = int(self.message_input.text)
#             url = "http://192.168.4.1/message"  # Replace with ESP32 IP address
            
#             # Send the integer as plain text
#             headers = {'Content-Type': 'text/plain'}
#             response = requests.post(url, data=str(message), headers=headers)
            
#             print("Response from ESP32:", response.text)
#         except ValueError:
#             print("Please enter a valid integer.")
#         except Exception as e:
#             print("Error:", e)
    
#     def update_plot(self, dt):
#         try:
#             # Request sensor and threshold data from ESP32
#             url = "http://192.168.4.1/random"  # Endpoint for sensor and threshold data
#             response = requests.get(url)
            
#             if response.status_code == 200:
#                 try:
#                     # Parse JSON response
#                     data = response.json()
#                     sensor_data = data.get("sensor", 0)
#                     threshold_data = data.get("threshold", 0)
#                 except ValueError:  # Handle non-JSON response
#                     print("Invalid JSON received:", response.text)
#                     return
                
#                 # Print sensor data to the terminal
#                 print(f"Sensor Data: {sensor_data}")
                
#                 # Update y-axis label to display the threshold value
#                 self.graph.ylabel = f"Thres: {threshold_data}"
                
#                 # Dynamically adjust y-axis limits if needed
#                 min_y = min(sensor_data, threshold_data)
#                 max_y = max(sensor_data, threshold_data)
#                 if min_y < self.graph.ymin:
#                     self.graph.ymin = min_y - 10  # Add margin
#                 if max_y > self.graph.ymax:
#                     self.graph.ymax = max_y + 10  # Add margin
                
#                 # Update plots
#                 if len(self.sensor_plot.points) > 20:  # Keep the last 20 points
#                     self.sensor_plot.points.pop(0)
#                 if len(self.threshold_plot.points) > 20:  # Keep the last 20 points
#                     self.threshold_plot.points.pop(0)
                
#                 # Append new points
#                 self.sensor_plot.points.append((self.time, sensor_data))
#                 self.threshold_plot.points.append((self.time, threshold_data))
                
#                 self.time += 1  # Increment the time for the next point
                
#                 # Adjust x-axis to move with new points after 20 points
#                 if self.time > 20:
#                     self.graph.xmin = self.time - 20
#                     self.graph.xmax = self.time
    
#             else:
#                 print("Failed to retrieve data from ESP32:", response.status_code)
        
#         except requests.RequestException as e:
#             print("Error updating plot:", e)




# if __name__ == '__main__':
#     KADAM().run()





