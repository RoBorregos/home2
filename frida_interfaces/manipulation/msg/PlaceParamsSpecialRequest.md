# Place Params Special Request
Message `PlaceParams` has a field for special requests. As of Robocup 2025, this field is populated with a JSON string that can be interpreted in several ways. Currently, the request consists of an object to put "close by" and a request to add more to it, such as left, right, on top of the object, etc. The request is made by the task manager and interpreted by the manipulation node.

## Example of a special request

```json
{
    "request": "close_by",
    "object": "bottle",
    "position": "left"
}
```