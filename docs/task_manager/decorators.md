# Decorators

## Mockable
This decorator is used to return a default value for a function.
This is important when mocking individual functions or a complete area.
In task managers, when an area is set to mock, all functions in that area will be mocked even if their decorator mock value is false.

### Parameters
Arguments:
- `return_value`: 

The value to return when the function is called. 

Default is void.

- `delay`:

The time to wait before returning the value. 

Default is 0 seconds.

- `mock`: 

If true, the function will be mocked and the default value will be returned.

Default is false.

### Implementation
```python
@mockable(return_value=<default_value>, delay=(int)<delay-seconds>, mock=(bool)<mock>)
```
Example
```python
@mockable(return_value="Success", delay=2, mock=True) 
def do_something(self):
    """<docstring>"""
    # function implementation
```

## Service Check
This decorator will internally wait for a service to check if it is available and return a default value if it is not.
When not implemented, if a task manager is running and a service is not available, the task will get stuck without any indication. Therefore, this decorator is important to ensure that the task manager does not get stuck and can return an error value.

### Parameters
Arguments:
- `client`:

The name of the client to check. (It can be a service or a action client)

- `default_value`:

The value to return when the service is not available.
Default is None.

- `timeout`:

The time set to wait for service.
Default is 3 seconds.

### Implementation

```python
@service_check(client="client_name", default_value=<default_value>, timeout=(int)<timeout-seconds>)
```

Example
```python
@service_check(client="client_name", default_value="Error", timeout=2)
def do_something(self):
    """<docstring>"""
    # function implementation
```

## Example
*If using both, the mockable should come first:

```python
@mockable(return_value=Status.EXECUTION_SUCCESS, delay=2)
@service_check("track_person_client", Status.EXECUTION_ERROR, TIMEOUT)
def track_person(self, track: bool = True):
    """Track the person in the image"""
    Logger.info(self.node, "Tracking person")
    request = SetBool.Request()
    request.data = track

    try:
        future = self.track_person_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=TIMEOUT)
        result = future.result()

        if not result.success:
            Logger.warn(self.node, "No person found")
            return Status.TARGET_NOT_FOUND

    except Exception as e:
        Logger.error(self.node, f"Error tracking person: {e}")
        return Status.EXECUTION_ERROR

    Logger.success(self.node, "Person tracking success")
    return Status.EXECUTION_SUCCESS
```