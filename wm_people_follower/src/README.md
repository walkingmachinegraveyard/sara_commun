# WM People Follower

This node generate move_base goal to follow someone.

## Service

### Request

```
int8 request
int8 ACQUIRE_TARGET=1
int8 START_FOLLOWING=2
int8 STOP_FOLLOWING=3
int8 RELEASE_TARGET=4
```

### Response

```
int8 response
int8 SUCCESS=1
int8 FAILURE=-1
```
