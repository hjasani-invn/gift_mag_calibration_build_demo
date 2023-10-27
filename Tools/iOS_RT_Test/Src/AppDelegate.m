//
//  AppDelegate.m
//  temp
//
//  Created by Vladimir Pentyukhov on 17/07/2017.
//  Copyright © 2017 Vladimir Pentyukhov. All rights reserved.
//

#import "AppDelegate.h"

#import "WrapperiOS.h"

@interface AppDelegate ()
{
#if 0
    int  mapViewWidth;
    int  mapViewHeight;
    
    CGPoint location;
    CGPoint previousLocation;
    
    double mapWidth;                      // meters
    double mapHeight;
    
    double left;                       // meters
    double down;                       // meters
    
    double mmcellsize;
    double mgcellsize;
    double mapcellsize;
    
    float magBias1;
    float magBias2;
    float magBias3;
    long long magBiasTime;
    
    int currentfloor;
    double currentX;
    double currentY;
    
    int minfloor;
    int maxfloor;
    
    double  scaleFactor ;
    
    double scaleStepperValueOld;
    
    NSString *mapBase;
    NSString *vizBase;
    
    NSString *trkFile;
    NSString *accuracyFile;
    
    
    NSString *jsonName;
    NSMutableArray *filesList;
    
    NSMutableData		  *receivedData;
    
    BOOL enableMap;
    NSFileHandle *fileTrk ;
    NSFileHandle *fileAccuracy ;
    NSString *documentDir;
    
    //UIBackgroundTaskIdentifier _backgroundRecordingID;
    
    NSTimer *backgroundTimer;
    
    BOOL  navEngineIsWorked;
    
    BOOL   fileIsClosed;
    
    
#endif
    
       WrapperiOS *wrapperiOS;
}
@end


int mainiOS( const int argc, const char** argv );

@implementation AppDelegate


- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions {
    // Override point for customization after application launch.
    
#if 0
    // Do any additional setup after loading the view, typically from a nib.
    NSArray  *docList = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    documentDir  = [docList objectAtIndex:0];
    documentDir = [documentDir stringByAppendingString:@"/"];
    NSLog(@"documentDir %@", documentDir);
    
    
    [[NSUserDefaults standardUserDefaults] setObject: jsonName forKey:@"Map"];
    NSString *mName = [jsonName stringByReplacingOccurrencesOfString:@".json" withString:@""];
    
    dispatch_async(dispatch_get_main_queue(), ^{
        //self.mapNameLabel.text = mName;
    });
    
    NSString *jsonFile  = [documentDir stringByAppendingString:jsonName];
    NSString* jsonString = [NSString stringWithContentsOfFile:jsonFile
                                                     encoding:NSUTF8StringEncoding
                                                        error:NULL];
    
    jsonString = [jsonString stringByReplacingOccurrencesOfString:@"\n" withString:@""];
    
    NSError *e = nil;
    
    // Create SBJSON object to parse JSON
    //SBJSON *parser = [[SBJSON alloc] init];
    // Parse JSON string into dictionary
    //NSDictionary *json = [parser objectWithString:jsonString error:&e];
    
    char *command_line[2];
    command_line[0] = "--settings";
        command_line[1] = "example.json";
    mainiOS( 2, command_line );
#endif
    
    
    wrapperiOS = [[WrapperiOS alloc] init];
    
    return YES;
}


- (void)applicationWillResignActive:(UIApplication *)application {
    // Sent when the application is about to move from active to inactive state. This can occur for certain types of temporary interruptions (such as an incoming phone call or SMS message) or when the user quits the application and it begins the transition to the background state.
    // Use this method to pause ongoing tasks, disable timers, and invalidate graphics rendering callbacks. Games should use this method to pause the game.
}


- (void)applicationDidEnterBackground:(UIApplication *)application {
    // Use this method to release shared resources, save user data, invalidate timers, and store enough application state information to restore your application to its current state in case it is terminated later.
    // If your application supports background execution, this method is called instead of applicationWillTerminate: when the user quits.
}


- (void)applicationWillEnterForeground:(UIApplication *)application {
    // Called as part of the transition from the background to the active state; here you can undo many of the changes made on entering the background.
}


- (void)applicationDidBecomeActive:(UIApplication *)application {
    // Restart any tasks that were paused (or not yet started) while the application was inactive. If the application was previously in the background, optionally refresh the user interface.
}


- (void)applicationWillTerminate:(UIApplication *)application {
    // Called when the application is about to terminate. Save data if appropriate. See also applicationDidEnterBackground:.
}


@end
